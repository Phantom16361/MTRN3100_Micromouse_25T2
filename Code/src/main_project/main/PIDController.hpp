#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <Arduino.h>

class PIDController {
public:
  PIDController(float kp=0.f, float ki=0.f, float kd=0.f);

  // Config
  void setGains(float kp, float ki, float kd);
  void setOutputLimits(float minVal, float maxVal);
  void setDerivativeSmoothing(float smoothingAlpha);     // 0..1 (EMA on D term)
  void setUseDerivativeOnMeasurement(bool enable);       // true = D on measurement, false = D on error
  void setFixedDt(float dt) { fixedDt = (dt > 0.f) ? dt : fixedDt; }

  // State
  void reset();                     // full reset
  void reset(float currentMeas);    // seeds lastMeasurement to avoid D-kick

  // Control
  float compute(float error, float measurement, float dt);  // preferred
  float compute(float error, float measurement) {           // convenience (uses fixedDt)
    return compute(error, measurement, fixedDt);
  }

private:
  // Gains
  float Kp, Ki, Kd;

  // Limits
  float outMin = -255.f, outMax = 255.f;

  // Internal state
  float integral = 0.f;
  float prevError = 0.f;
  float lastMeasurement = 0.f;

  // Derivative filtering
  float alpha = 0.1f;               // EMA smoothing factor for D
  float filtD = 0.f;
  bool  dOnMeas = true;             // default matches your main.ino call

  // For 2-arg compute()
  float fixedDt = 0.01f;
};

#endif
