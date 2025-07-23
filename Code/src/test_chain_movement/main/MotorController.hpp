/**************************************************************
 *  File         : MotorController.hpp
 *  Author       : Jason E Tomczyk
 *  Description  : Header for DRV8835 motor interface.
 *                 Allows setting PWM and direction pins
 *                 for independent left/right wheel control.
 * 
 *                 Do not edit unless discussed.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Declared setMotorPWM() and internal setMotor()
 *             for pin-level actuation of motor driver.
 *************************************************************/

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <Arduino.h>
#include "pin_config.hpp"

class MotorController {
public:
    MotorController();
    void begin();

    // Set PWM for left and right motors (range: -255 to 255)
    void setMotorPWM(int leftPWM, int rightPWM);

private:
    void setMotor(int pwm, int pwmPin, int dirPin);
};

#endif // MOTOR_CONTROLLER_HPP
