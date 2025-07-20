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

// ———— Instantiate the yaw‐PID controller with those gains ————
PIDController yawPID(KP, KI, KD);

// ———— PWM limits & small‐drift cutoff ————
const int   MAX_PWM       =  60;   // ±60 PWM clamp
const int   MIN_PWM_DEAD  =  15;   // overcome stiction
const float DRIFT_DEG     =   1.0f;// within ±1° → treat as zero

// ———— Yaw references & timing ————
float initialYaw, setpointYaw;
unsigned long lastTime = 0;

// ———— Hardware ————
MotorController motor;
IMUOdometry      imu;

// ———— Wrap an angle into (–180, 180] ————
static float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("=== Continuous CW→90° correction ===");
  Wire.begin();

  // init IMU
  imu.begin(/*gyroCfg=*/1, /*accCfg=*/0);
  delay(100);
  imu.update();

  // init motors
  motor.begin();

  // compute & announce setpoint
  initialYaw  = imu.getYawDegrees();
  setpointYaw = wrap180(initialYaw + 90.0f);
  Serial.print("Start yaw: "); Serial.println(initialYaw,1);
  Serial.print("90° setpt: "); Serial.println(setpointYaw,1);

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

  // read & wrap current yaw
  imu.update();
  float rawYaw = imu.getYawDegrees();
  float error  = wrap180(setpointYaw - rawYaw);

  // let PID fight drift
  float u = yawPID.compute(error, dt, rawYaw);
  int   pwm = (int)u;

  // if error is very small, cut power entirely
  if (fabs(error) < DRIFT_DEG) {
    pwm = 0;
  }
  // else, ensure we overcome stiction
  else if (pwm > 0 && pwm < MIN_PWM_DEAD) {
    pwm = MIN_PWM_DEAD;
  }
  else if (pwm < 0 && pwm > -MIN_PWM_DEAD) {
    pwm = -MIN_PWM_DEAD;
  }

  // clamp safety
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  // always rotate CLOCKWISE toward setpoint:
  // motor.setMotorPWM(left, right) where positive left/negative right = CCW
  // so invert to get CW
  motor.setMotorPWM(-pwm, +pwm);
}
