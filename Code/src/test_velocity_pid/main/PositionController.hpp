#pragma once

class PositionController {
public:
    PositionController(float kp, float ki, float kd);
    void setTarget(float targetMM);
    float update(float currentPosMM, float dt);
    bool isFinished() const;
    void reset();

private:
    float target;
    float kp, ki, kd;
    float integral;
    float lastError;
    bool firstRun;
};
