/**************************************************************
 *  File         : MotorController.cpp
 *  Author       : Jason E Tomczyk
 *  Description  : Low-level PWM and direction output control
 *                 for DRV8835 motor driver. Used to command
 *                 left/right motors directly.
 * 
 *                 Do not edit unless discussed.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Finalized basic control structure for left/right
 *             motors with PWM and direction settings.
 *************************************************************/

#include "MotorController.hpp"

MotorController::MotorController() {}

void MotorController::begin() {
    pinMode(MOT1_PWM, OUTPUT);
    pinMode(MOT1_DIR, OUTPUT);
    pinMode(MOT2_PWM, OUTPUT);
    pinMode(MOT2_DIR, OUTPUT);

    setMotorPWM(0, 0); // Stop motors initially
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
