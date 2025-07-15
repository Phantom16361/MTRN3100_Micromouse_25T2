
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float minVal, float maxVal);
    void setDerivativeSmoothing(float alpha);
    void reset();
    float compute(float error, float dt, float measurement);

    // Optional behavior toggles
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

    // Modular control
    float lastTargetSetpoint = 0.0;
    bool freezeDWhenSPZero = false;
    float deadband = 0.0;
    bool useDerivativeOnMeasurement = false;
    float lastMeasurement = 0.0;
};

#endif // PID_CONTROLLER_HPP
