// PIDController.hpp

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float minVal, float maxVal);
    void setDerivativeSmoothing(float alpha);

    // Reset internal history; optionally seed the D‐filter
    void reset(float currentMeasurement = 0.0f);

    float compute(float error, float dt, float measurement);

    // (These are unused in current sketch, but available)
    void setTargetSetpoint(float sp);
    void enableDerivativeFreezeOnZeroSP(bool enable);
    void setVelocityDeadband(float threshold);
    void setUseDerivativeOnMeasurement(bool enable);

private:
    float Kp, Ki, Kd;
    float integral;
    float previousError;
    float filteredDerivative;
    float outputMin, outputMax;
    float alpha;

    float lastTargetSetpoint;
    bool  freezeDWhenSPZero;
    float deadband;
    bool  useDerivativeOnMeasurement;
    float lastMeasurement;
};

#endif // PID_CONTROLLER_HPP
