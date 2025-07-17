/**************************************************************
 *  File         : main.ino
 *  Author       : Jason E Tomczyk
 *  Description  : Velocity PID test harness for verifying wheel
 *                 speed response. Logs target vs actual velocity,
 *                 applies factory-initialized PID outputs to motors.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Initial straight-line velocity tracking test using
 *             PIDController::Left/Right and EncoderOdometry feedback.
 *************************************************************/


#include <Arduino.h>
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PIDController.hpp"

// ─────────────────────────────────────────────
// Global instances
// ─────────────────────────────────────────────

EncoderOdometry odom;
MotorController motor;

PIDController leftPID  = PIDController::Left();
PIDController rightPID = PIDController::Right();

// ─────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────

float targetSpeedL = 0.0f;
float targetSpeedR = 0.0f;

unsigned long lastControlTime = 0;

// ─────────────────────────────────────────────
// Arduino setup()
// ─────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  odom.begin();
  motor.begin();

  // Set test target speeds (adjust as needed)
  targetSpeedL = 100.0f;  // mm/s
  targetSpeedR = 100.0f;
}

// ─────────────────────────────────────────────
// Arduino loop()
// ─────────────────────────────────────────────

void loop() {
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = now;

    odom.update();

    float actualL = odom.getLeftSpeedMMs();
    float actualR = odom.getRightSpeedMMs();

    float errorL = targetSpeedL - actualL;
    float errorR = targetSpeedR - actualR;

    float pwmL = leftPID.compute(errorL, actualL);
    float pwmR = rightPID.compute(errorR, actualR);

    motor.setMotorPWM(pwmL, pwmR);

    // Debug
    // Serial plotter output
    Serial.print("Target_L:"); Serial.print(targetSpeedL, 2); Serial.print(" ");
    Serial.print("Speed_L:"); Serial.print(actualL, 2); Serial.print(" ");
    Serial.print("PWM_L:"); Serial.print(pwmL, 2); Serial.print(" ");
    Serial.print("Target_R:"); Serial.print(targetSpeedR, 2); Serial.print(" ");
    Serial.print("Speed_R:"); Serial.print(actualR, 2); Serial.print(" ");
    Serial.print("PWM_R:"); Serial.print(pwmR, 2); Serial.print(" ");
    Serial.print("REF_Bottom:-100 "); Serial.println("REF_Top:250");
  }
}
