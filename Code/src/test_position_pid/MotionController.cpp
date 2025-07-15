#include "MotionController.hpp"
#include <Arduino.h>

MotionController::MotionController()
    : odom(WHEEL_RADIUS_MM, AXLE_LENGTH_MM, TICKS_PER_REV),
      leftPID(LEFT_POS_KP, LEFT_POS_KI, LEFT_POS_KD),
      rightPID(RIGHT_POS_KP, RIGHT_POS_KI, RIGHT_POS_KD) {}

void MotionController::begin() {
    odom.begin();
    motor.begin();
    leftPID.setOutputLimits(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT);
    rightPID.setOutputLimits(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT);
}

void MotionController::moveForward(float distance_mm, float duration_s) {
    trajectory.generate(0.0f, 0.0f, distance_mm, 0.0f, duration_s);
    startX = odom.getX();
    startTime = millis();
    active = true;
}

void MotionController::update(unsigned long currentMillis) {
    odom.update();
    if (!active) return;

    float t = (currentMillis - startTime) / 1000.0f;
    if (t > trajectory.getDuration()) {
        motor.setMotorPWM(0, 0);
        active = false;
        return;
    }

    float currentX = odom.getX() - startX;
    float x_des = trajectory.getPosition(t);
    float v_des = trajectory.getVelocity(t);

    float leftVel = odom.getLeftSpeedMMs();
    float rightVel = odom.getRightSpeedMMs();

    float leftErr = v_des - leftVel;
    float rightErr = v_des - rightVel;

    float leftPWM = leftPID.compute(leftErr, t, leftVel);
    float rightPWM = rightPID.compute(rightErr, t, rightVel);

    motor.setMotorPWM(leftPWM, rightPWM);
}

float MotionController::getX() const {
    return odom.getX();
}

bool MotionController::isFinished() const {
    return !active;
}
