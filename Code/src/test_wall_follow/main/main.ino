#include <Arduino.h>
#include <Wire.h>

#include "pin_config.hpp"
#include "robot_param.hpp"
#include "MotorController.hpp"
#include "IMUOdometry.hpp"
#include "PIDController.hpp"

// ———— Hardware ————
MotorController motor;
IMUOdometry      imu;

// ———— PID (gentle) ————
PIDController yawPID(1.5f, 0.0f, 0.3f);
const int       MAX_PWM      =  60;    // ±60 PWM clamp
const int       MIN_PWM_DEAD =  15;    // minimum drive to overcome dead-zone
const float     TOL_DEG      =   5.0f; // within ±5° → hold

// ———— Yaw refs & timing ————
float initialYaw, setpointYaw;
unsigned long lastTime;

// wrap angle into [–180,180]
static float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("=== Continuous 90° correction loop ===");
  Wire.begin();

  // initialize IMU
  imu.begin(1, 0);
  delay(100);
  imu.update();

  // initialize motors
  motor.begin();

  // compute fixed +90° setpoint
  initialYaw  = imu.getYawDegrees();
  setpointYaw = wrap180(initialYaw + 90.0f);
  Serial.print("Start yaw: "); Serial.println(initialYaw,1);
  Serial.print("Target   : "); Serial.println(setpointYaw,1);

  // configure PID
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

  // read current yaw
  imu.update();
  float rawYaw = imu.getYawDegrees();

  // compute wrapped error in (–180,180]
  float error = wrap180(setpointYaw - rawYaw);

  // determine direction: normally by sign(error),
  // but if near ±180°, choose direction based on rawYaw vs setpointYaw
  int dir;
  if (fabs(fabs(error) - 180.0f) < 1.0f) {
    dir = (rawYaw < setpointYaw) ? +1 : -1;
  } else {
    dir = (error > 0.0f) ? +1 : -1;
  }

  // compute PID on magnitude of error
  float u = yawPID.compute(fabs(error), dt, rawYaw);
  int rawPwm = (int)u;

  // enforce dead-zone only when outside tolerance
  int pwm;
  if (fabs(error) < TOL_DEG) {
    // within tolerance → hold still
    pwm = 0;
  } else if (abs(rawPwm) < MIN_PWM_DEAD) {
    // small command → bump to overcome stiction
    pwm = MIN_PWM_DEAD;
  } else {
    pwm = abs(rawPwm);
  }

  // drive motors: dir=+1 → CCW, dir=-1 → CW
  motor.setMotorPWM(dir * pwm, -dir * pwm);
}
