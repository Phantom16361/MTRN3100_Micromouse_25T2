
#include <Arduino.h>
#include "PIDController.hpp"

PIDController::PIDController(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd),
      integral(0), previousError(0),
      outputMin(-255), outputMax(255),
      filteredDerivative(0), alpha(0.1f),
      lastMeasurement(0.0f) {}

void PIDController::setGains(float kp, float ki, float kd) {
    Kp = kp; Ki = ki; Kd = kd;
}

void PIDController::setOutputLimits(float minVal, float maxVal) {
    outputMin = minVal;
    outputMax = maxVal;
}

void PIDController::setDerivativeSmoothing(float smoothingAlpha) {
    alpha = constrain(smoothingAlpha, 0.0f, 1.0f);
}

void PIDController::reset() {
    integral = 0;
    previousError = 0;
    filteredDerivative = 0;
    lastMeasurement = 0;
}

void PIDController::setTargetSetpoint(float sp) {
    lastTargetSetpoint = sp;
}

void PIDController::enableDerivativeFreezeOnZeroSP(bool enable) {
    freezeDWhenSPZero = enable;
}

void PIDController::setVelocityDeadband(float threshold) {
    deadband = threshold;
}

void PIDController::setUseDerivativeOnMeasurement(bool enable) {
    useDerivativeOnMeasurement = enable;
}

float PIDController::compute(float error, float dt, float measurement) {
    if (dt <= 0.0f) return 0;

    if (abs(error) < deadband) {
        error = 0.0;
    }

    integral += error * dt;

    float rawDerivative;
    if (useDerivativeOnMeasurement) {
        rawDerivative = (measurement - lastMeasurement) / dt;
        lastMeasurement = measurement;
    } else {
        rawDerivative = (error - previousError) / dt;
        previousError = error;
    }

    if (freezeDWhenSPZero && abs(lastTargetSetpoint) < 1.0f) {
        filteredDerivative = 0.0f;
    } else {
        filteredDerivative = alpha * rawDerivative + (1.0f - alpha) * filteredDerivative;
    }

    float output = Kp * error + Ki * integral - Kd * filteredDerivative;

    if (output > outputMax) output = outputMax;
    else if (output < outputMin) output = outputMin;

    return output;
}
