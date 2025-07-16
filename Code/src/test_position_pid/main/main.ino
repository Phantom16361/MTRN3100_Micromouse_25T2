#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PIDController.hpp"
#include "CubicTrajectory.hpp"

EncoderOdometry odom;
MotorController motor;

PIDController positionPID(LEFT_POS_KP, LEFT_POS_KI, LEFT_POS_KD);  // Tune these

CubicTrajectory traj;

unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL_MS = 25;

float duration = 0.6;   // seconds
unsigned long startTime = 0;
bool reachedTarget = false;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Direct Position PID Test (with Position Error Stop)");

  odom.begin();
  motor.begin();

  traj.generate(0, 0, 200.0, 0, duration); // Move forward 120mm

  positionPID.setOutputLimits(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT);
  startTime = millis();
}

void loop() {
  odom.update();

  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    float currentTime = (now - startTime) / 1000.0;
    float x = odom.getX();
    float x_des = traj.getPosition(min(currentTime, duration));  // Clamp time to duration
    float error = x_des - x;

    if (!reachedTarget) {
      float pwm = positionPID.compute(error, dt, x);
      motor.setMotorPWM(pwm, pwm);

      if (currentTime > duration && abs(error) < 1.5) {
        reachedTarget = true;
        motor.setMotorPWM(0, 0);
        Serial.println("✔ Reached Target Position");
      }

      Serial.print("T:"); Serial.print(currentTime, 2); Serial.print(" ");
      Serial.print("X_DES:"); Serial.print(x_des, 1); Serial.print(" ");
      Serial.print("X:"); Serial.print(x, 1); Serial.print(" ");
      Serial.print("ERR:"); Serial.print(error, 1); Serial.print(" ");
      Serial.print("PWM:"); Serial.println(pwm, 1);
    }
  }
}
