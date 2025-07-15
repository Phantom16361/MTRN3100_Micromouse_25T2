
#include "PIDController.hpp"

PIDController::PIDController(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd),
      integral(0), previousError(0),
      outputMin(-255), outputMax(255) {}

void PIDController::setGains(float kp, float ki, float kd) {
    Kp = kp; Ki = ki; Kd = kd;
}

void PIDController::setOutputLimits(float minVal, float maxVal) {
    outputMin = minVal;
    outputMax = maxVal;
}

void PIDController::reset() {
    integral = 0;
    previousError = 0;
}

float PIDController::compute(float error, float dt) {
    integral += error * dt;
    float derivative = (error - previousError) / dt;
    previousError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    if (output > outputMax) output = outputMax;
    else if (output < outputMin) output = outputMin;

    return output;
}
