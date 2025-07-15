
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
