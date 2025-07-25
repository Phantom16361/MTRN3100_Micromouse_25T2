// PIDController.cpp

#include <Arduino.h>
#include "PIDController.hpp"

PIDController::PIDController(float kp, float ki, float kd)
  : Kp(kp), Ki(ki), Kd(kd),
    integral(0.0f),
    previousError(0.0f),
    filteredDerivative(0.0f),
    outputMin(-255), outputMax(255),
    alpha(0.1f),
    lastTargetSetpoint(0.0f),
    freezeDWhenSPZero(false),
    deadband(0.0f),
    useDerivativeOnMeasurement(false),
    lastMeasurement(0.0f)
{}

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

void PIDController::reset(float currentMeasurement) {
    integral           = 0.0f;
    previousError      = 0.0f;
    filteredDerivative = 0.0f;
    lastMeasurement    = currentMeasurement;
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
    if (dt <= 0.0f) return 0.0f;

    if (fabs(error) < deadband) {
        error = 0.0f;
    }

    float rawDerivative;
    if (useDerivativeOnMeasurement) {
        rawDerivative   = (measurement - lastMeasurement) / dt;
        lastMeasurement = measurement;
    } else {
        rawDerivative    = (error - previousError) / dt;
        previousError    = error;
    }

    if (freezeDWhenSPZero && fabs(lastTargetSetpoint) < 1e-3f) {
        filteredDerivative = 0.0f;
    } else {
        filteredDerivative = alpha * rawDerivative
                             + (1.0f - alpha) * filteredDerivative;
    }

    float output = Kp * error
                 - Kd * filteredDerivative;

    return constrain(output, outputMin, outputMax);
}