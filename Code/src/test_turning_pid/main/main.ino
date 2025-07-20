// main.ino

#include <Arduino.h>
#include <Wire.h>

#include "pin_config.hpp"
#include "robot_param.hpp"
#include "MotorController.hpp"
#include "IMUOdometry.hpp"
#include "PIDController.hpp"

// ———— PID gains & integral to counter drift ————
const float KP = 1.5f;
const float KI = 0.1f;
const float KD = 0.3f;

// ———— Instantiate yaw PID ————
PIDController yawPID(KP, KI, KD);

// ———— PWM & thresholds ————
const int   MAX_PWM           =  60;   // ±60 PWM clamp
const int   MIN_PWM_DEAD      =  15;   // overcome stiction
const float DRIFT_DEG         =   1.0f;// within ±1° → zero output
const float INIT_TOL          =   3.0f;// initial phase tolerance
const int   INIT_PWM_ANTICW   =  30;   // initial anticlockwise turn speed

// ———— Phase flag & yaw refs ————
static bool firstMove = true;
float initialYaw, setpointYaw;
unsigned long lastTime = 0;

// ———— Hardware ————
MotorController motor;
IMUOdometry      imu;

// wrap angle into (–180,180]
static float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("=== Initial clockwise 90° ===");
  Wire.begin();

  // init IMU
  imu.begin(1, 0);
  delay(100);
  imu.update();

  // init motors
  motor.begin();

  // record initial yaw and compute setpoint (+90°)
  initialYaw  = imu.getYawDegrees();
  setpointYaw = wrap180(initialYaw - 90.0f);
  Serial.print("Start yaw: "); Serial.println(initialYaw,1);
  Serial.print("Target   : "); Serial.println(setpointYaw,1);

  // configure PID
  yawPID.setGains(KP, KI, KD);
  yawPID.setOutputLimits(-MAX_PWM, MAX_PWM);
  yawPID.setDerivativeSmoothing(0.1f);
  yawPID.setUseDerivativeOnMeasurement(true);
  yawPID.reset(initialYaw);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) * 0.001f;
  lastTime = now;

  // update yaw
  imu.update();
  float rawYaw = imu.getYawDegrees();

  // PHASE 1: rough turn to setpoint
  if (firstMove) {
    // compute shortest‐path error (±180°)
    float error = wrap180(setpointYaw - rawYaw);

    // if we're still outside the initial tolerance, keep spinning
    if (fabs(error) > INIT_TOL) {
      // pick direction: +error → CCW, -error → CW
      int dir = (error > 0) ? 1 : -1;
      // left=-dir*speed, right=+dir*speed → CCW when dir=+1, CW when dir=-1
      motor.setMotorPWM(-dir * INIT_PWM_ANTICW,
                         dir * INIT_PWM_ANTICW);
    } else {
      // close enough—stop and hand off to PID
      motor.setMotorPWM(0, 0);
      firstMove = false;
      yawPID.reset(rawYaw);
    }
    return;
  }

  // PHASE 2: PID‐based fine correction
  float error = wrap180(setpointYaw - rawYaw);
  float u     = yawPID.compute(error, dt, rawYaw);
  int pwm     = (int)u;

  // deadband
  if (fabs(error) < DRIFT_DEG) {
    pwm = 0;
  } else if (pwm > 0 && pwm < MIN_PWM_DEAD) {
    pwm = MIN_PWM_DEAD;
  } else if (pwm < 0 && pwm > -MIN_PWM_DEAD) {
    pwm = -MIN_PWM_DEAD;
  }

  // clamp
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  // drive: -pwm/+pwm → CCW when pwm>0, CW when pwm<0
  motor.setMotorPWM(-pwm, +pwm);
}
