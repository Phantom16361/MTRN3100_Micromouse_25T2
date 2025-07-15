
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float minVal, float maxVal);
    void reset();
    float compute(float error, float dt);

private:
    float Kp, Ki, Kd;
    float integral;
    float previousError;
    float outputMin, outputMax;
};

#endif // PID_CONTROLLER_HPP
