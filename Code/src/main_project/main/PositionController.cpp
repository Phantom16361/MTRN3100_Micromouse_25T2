#include "PositionController.hpp"
#include <Arduino.h>

PositionController::PositionController(float kp_, float ki_, float kd_)
  : kp(kp_), ki(ki_), kd(kd_), target(0), integral(0), lastError(0), firstRun(true) {}

void PositionController::setTarget(float targetMM) {
    target = targetMM;
    integral = 0;
    lastError = 0;
    firstRun = true;
}

float PositionController::update(float currentPosMM, float dt) {
    float error = target - currentPosMM;
    integral += error * dt;

    float derivative = 0;
    if (!firstRun) {
        derivative = (error - lastError) / dt;
    } else {
        firstRun = false;
    }

    lastError = error;

    float output = kp * error + ki * integral + kd * derivative;
    return output;
}

bool PositionController::isFinished() const {
    return abs(lastError) < 1.5;  // within 1.5 mm
}

void PositionController::reset() {
    integral = 0;
    lastError = 0;
    firstRun = true;
}
