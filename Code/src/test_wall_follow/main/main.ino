// main.ino

#include <Arduino.h>
#include <Wire.h>

#include "pin_config.hpp"
#include "robot_param.hpp"
#include "MotorController.hpp"
#include "IMUOdometry.hpp"
#include "PIDController.hpp"

// ———— States ————
enum State { ROTATE, WAIT };
State state = ROTATE;

// ———— Hardware ————
MotorController motor;
IMUOdometry      imu;

// ———— PID (slower, gentler) ————
PIDController yawPID(1.5f, 0.0f, 0.3f);
const int       MAX_PWM       =  60;   // clamp PWM ±60
const int       MIN_PWM_DEAD  =  15;   // dead‐zone

// ———— Parameters ————
const float   TOL_DEG       =  5.0f;    // ±5° settle
const float   DETECT_THRESH = 15.0f;    // manual twist >15°
const unsigned long STABLE_MS = 200UL;  // must hold within tol 200 ms

// ———— Yaw refs & timing ————
float initialYaw, setpointYaw;
unsigned long lastTime = 0, stableTime = 0;

// wrap angle into [–180,180]
static float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("=== Rotate → Twist → Return (loop) ===");

  Wire.begin();
  imu.begin(1,0);
  delay(100);
  imu.update();

  motor.begin();

  initialYaw  = imu.getYawDegrees();
  setpointYaw = wrap180(initialYaw + 90.0f);
  Serial.print("Start yaw: "); Serial.println(initialYaw,1);
  Serial.print("Target   : "); Serial.println(setpointYaw,1);

  yawPID.setOutputLimits(-MAX_PWM, MAX_PWM);
  yawPID.setDerivativeSmoothing(0.1f);
  yawPID.setUseDerivativeOnMeasurement(true);
  yawPID.reset(initialYaw);

  lastTime   = millis();
  stableTime = 0;
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) * 0.001f;
  lastTime = now;

  imu.update();
  float rawYaw = imu.getYawDegrees();
  float error  = wrap180(setpointYaw - rawYaw);

  switch (state) {
    case ROTATE:
      if (fabs(error) > TOL_DEG) {
        // still moving toward setpoint
        stableTime = 0;
        float u = yawPID.compute(error, dt, rawYaw);
        int pwm = (int)u;
        if (abs(pwm) < MIN_PWM_DEAD) pwm = 0;
        motor.setMotorPWM(pwm, -pwm);
      } else {
        // within tolerance → wait for STABLE_MS before declaring settled
        if (stableTime == 0) stableTime = now;
        else if (now - stableTime >= STABLE_MS) {
          motor.setMotorPWM(0,0);
          static bool firstCycle = true;
          if (firstCycle) {
            Serial.println("✓ Reached +90°");
            firstCycle = false;
          } else {
            Serial.println("✓ Returned to +90°");
          }
          yawPID.reset(rawYaw);
          state = WAIT;
          stableTime = 0;
        }
      }
      break;

    case WAIT:
      // sitting at +90°, waiting for manual twist
      if (fabs(wrap180(rawYaw - setpointYaw)) > DETECT_THRESH) {
        Serial.print("Twist detected: yaw="); Serial.println(rawYaw,1);
        yawPID.reset(rawYaw);
        state = ROTATE;
      }
      break;
  }
}
