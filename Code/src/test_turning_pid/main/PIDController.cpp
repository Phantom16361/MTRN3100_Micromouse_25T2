#include "PIDController.hpp"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd) {}

PIDController PIDController::Left() {
    PIDController pid(LEFT_VEL_KP, LEFT_VEL_KI, LEFT_VEL_KD);
    pid.setOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    pid.setDerivativeSmoothing(PID_DERIV_SMOOTH);
    pid.setVelocityDeadband(PID_DEADBAND);
    pid.enableDerivativeFreezeOnZeroSP(true);
    pid.setUseDerivativeOnMeasurement(true);
    return pid;
}

PIDController PIDController::Right() {
    PIDController pid(RIGHT_VEL_KP, RIGHT_VEL_KI, RIGHT_VEL_KD);
    pid.setOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    pid.setDerivativeSmoothing(PID_DERIV_SMOOTH);
    pid.setVelocityDeadband(PID_DEADBAND);
    pid.enableDerivativeFreezeOnZeroSP(true);
    pid.setUseDerivativeOnMeasurement(true);
    return pid;
}

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

void PIDController::enableDerivativeFreezeOnZeroSP(bool enable) {
    freezeDWhenSPZero = enable;
}

void PIDController::setVelocityDeadband(float threshold) {
    deadband = threshold;
}

void PIDController::setUseDerivativeOnMeasurement(bool enable) {
    useDerivativeOnMeasurement = enable;
}

void PIDController::setTargetSetpoint(float sp) {
    lastTargetSetpoint = sp;
}

// @param CONTROL_DT = time delta in seconds
float PIDController::compute(float error, float measurement) {
    if (CONTROL_DT <= 0.0f) return 0;

    if (abs(error) < deadband) {
        error = 0.0f;
    }

    integral += error * CONTROL_DT;

    float rawDerivative;
    if (useDerivativeOnMeasurement) {
        rawDerivative = (measurement - lastMeasurement) / CONTROL_DT;
        lastMeasurement = measurement;
    } else {
        rawDerivative = (error - previousError) / CONTROL_DT;
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
