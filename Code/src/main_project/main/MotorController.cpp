#include <Arduino.h>
#include "MotorController.hpp"

MotorController::MotorController() {}

void MotorController::begin() {
    pinMode(MOT1_PWM, OUTPUT);
    pinMode(MOT1_DIR, OUTPUT);
    pinMode(MOT2_PWM, OUTPUT);
    pinMode(MOT2_DIR, OUTPUT);
    setMotorPWM(0, 0);
}

void MotorController::setMotorPWM(int leftPWM, int rightPWM) {
    setMotor(leftPWM, MOT1_PWM, MOT1_DIR);
    setMotor(rightPWM, MOT2_PWM, MOT2_DIR);
}

void MotorController::setMotor(int pwm, int pwmPin, int dirPin) {
    bool direction = pwm >= 0;
    pwm = constrain(abs(pwm), 0, 255);
    digitalWrite(dirPin, direction ? HIGH : LOW);
    analogWrite(pwmPin, pwm);
}
