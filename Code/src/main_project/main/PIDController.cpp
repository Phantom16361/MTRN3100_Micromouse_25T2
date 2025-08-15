#include "PIDController.hpp"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd)
: Kp(kp), Ki(ki), Kd(kd) {}

void PIDController::setGains(float kp, float ki, float kd) { Kp = kp; Ki = ki; Kd = kd; }

void PIDController::setOutputLimits(float minVal, float maxVal) {
  outMin = minVal; outMax = maxVal;
  if (outMin > outMax) { float t = outMin; outMin = outMax; outMax = t; }
}

void PIDController::setDerivativeSmoothing(float smoothingAlpha) {
  // clamp 0..1
  alpha = (smoothingAlpha < 0.f) ? 0.f : (smoothingAlpha > 1.f ? 1.f : smoothingAlpha);
}

void PIDController::setUseDerivativeOnMeasurement(bool enable) { dOnMeas = enable; }

void PIDController::reset() {
  integral = 0.f;
  prevError = 0.f;
  filtD = 0.f;
  lastMeasurement = 0.f;
}

void PIDController::reset(float currentMeas) {
  integral = 0.f;
  prevError = 0.f;
  filtD = 0.f;
  lastMeasurement = currentMeas;
}

float PIDController::compute(float error, float measurement, float dt) {
  if (dt <= 0.f) return 0.f;

  // I term
  integral += error * dt;

  // D term (on measurement or error)
  float rawD;
  if (dOnMeas) {
    rawD = (measurement - lastMeasurement) / dt;
    lastMeasurement = measurement;
  } else {
    rawD = (error - prevError) / dt;
    prevError = error;
  }

  // EMA smoothing on D
  filtD = alpha * rawD + (1.f - alpha) * filtD;

  // PID sum (note: D on measurement subtracts Kd*d(meas)/dt, equivalent to -Kd * filtD)
  float u = Kp * error + Ki * integral - Kd * filtD;

  // Clamp
  if (u > outMax) u = outMax;
  else if (u < outMin) u = outMin;

  return u;
}
