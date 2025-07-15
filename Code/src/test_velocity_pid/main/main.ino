
#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PIDController.hpp"

EncoderOdometry odom(WHEEL_RADIUS_MM, AXLE_LENGTH_MM, TICKS_PER_REV);
MotorController motor;

PIDController leftPID(1.85, 0.28, 0.025);
PIDController rightPID(1.85, 0.28, 0.02);

unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL_MS = 25;

float targetSpeedSet = 200.0;
float targetSpeedL = 0.0;
float targetSpeedR = 0.0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Looping Step Input PID Tuner");

  odom.begin();
  motor.begin();

  leftPID.setOutputLimits(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT);
  rightPID.setOutputLimits(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT);

  leftPID.setDerivativeSmoothing(0.1);
  rightPID.setDerivativeSmoothing(0.1);

  leftPID.setUseDerivativeOnMeasurement(true);
  rightPID.setUseDerivativeOnMeasurement(true);

  leftPID.enableDerivativeFreezeOnZeroSP(true);
  rightPID.enableDerivativeFreezeOnZeroSP(true);

  leftPID.setVelocityDeadband(0.1);
  rightPID.setVelocityDeadband(0.1);
}

void loop() {
  odom.update();
  unsigned long now = millis();

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    // 4s ON, 4s OFF step pattern
    if ((now / 1000) % 8 < 4) {
      targetSpeedL = targetSpeedSet;
      targetSpeedR = targetSpeedSet;
    } else {
      targetSpeedL = 0.0;
      targetSpeedR = 0.0;
    }

    float leftVel = odom.getLeftSpeedMMs();
    float rightVel = odom.getRightSpeedMMs();

    float errorL = targetSpeedL - leftVel;
    float errorR = targetSpeedR - rightVel;

    leftPID.setTargetSetpoint(targetSpeedL);
    rightPID.setTargetSetpoint(targetSpeedR);

    float leftPWM = leftPID.compute(errorL, dt, leftVel);
    float rightPWM = rightPID.compute(errorR, dt, rightVel);

    if (targetSpeedL == 0 && abs(leftVel) < 1.0) {
      leftPID.reset();
      leftPWM = 0;
    }

    if (targetSpeedR == 0 && abs(rightVel) < 1.0) {
      rightPID.reset();
      rightPWM = 0;
    }

    motor.setMotorPWM(leftPWM, rightPWM);

    // Serial plotter output
    Serial.print("L_SP:"); Serial.print(targetSpeedL, 2); Serial.print(" ");
    Serial.print("L_VEL:"); Serial.print(leftVel, 2); Serial.print(" ");
    Serial.print("L_OUT:"); Serial.print(leftPWM, 2); Serial.print(" ");
    Serial.print("R_SP:"); Serial.print(targetSpeedR, 2); Serial.print(" ");
    Serial.print("R_VEL:"); Serial.print(rightVel, 2); Serial.print(" ");
    Serial.print("R_OUT:"); Serial.print(rightPWM, 2); Serial.print(" ");
    Serial.print("REF_Bottom:-100 "); Serial.println("REF_Top:250");
  }
}
