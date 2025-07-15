#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PIDController.hpp"

EncoderOdometry odom(WHEEL_RADIUS_MM, AXLE_LENGTH_MM, TICKS_PER_REV);
MotorController motor;

PIDController leftPID(1.55, 0.02, 0.01);   // Tweak gains
PIDController rightPID(1.59, 0.00, 0.0);

unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL_MS = 25;

float targetSpeedSet = 100.0;

float targetSpeedL = 0.0;
float targetSpeedR = 0.0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Looping Step Input PID Test");

  odom.begin();
  motor.begin();

  leftPID.setOutputLimits(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT);
  rightPID.setOutputLimits(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT);

}

void loop() {
  odom.update();

  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    // 6s ON (100 mm/s), 6s OFF (0 mm/s) loop
    if ((now / 1000) % 8 < 4) {
      targetSpeedL = targetSpeedR = targetSpeedSet;
    } else {
      targetSpeedL = targetSpeedR = 0;
    }

    float leftVel = odom.getLeftSpeedMMs();
    float rightVel = odom.getRightSpeedMMs();

    float leftError = targetSpeedL - leftVel;
    float rightError = targetSpeedR - rightVel;

    float leftPWM = leftPID.compute(leftError, dt);
    float rightPWM = rightPID.compute(rightError, dt);

    motor.setMotorPWM(leftPWM, rightPWM);

    // Serial Plotter Output
    Serial.print("L_SP:");      Serial.print(targetSpeedL, 2);  Serial.print(" ");
    Serial.print("L_VEL:");     Serial.print(leftVel, 2);       Serial.print(" ");
    Serial.print("L_OUT:");     Serial.print(leftPWM, 2);       Serial.print(" ");
    Serial.print("R_SP:");      Serial.print(targetSpeedR, 2);  Serial.print(" ");
    Serial.print("R_VEL:");     Serial.print(rightVel, 2);      Serial.print(" ");
    Serial.print("R_OUT:");     Serial.print(rightPWM, 2);      Serial.print(" ");
    Serial.print("REF_Bottom:"); Serial.print(-50);            Serial.print(" ");
    Serial.print("REF_Top:");    Serial.println(180);
  }
}
