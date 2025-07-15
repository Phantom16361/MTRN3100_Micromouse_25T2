#pragma once

class CubicTrajectory {
public:
    void generate(float x0, float v0, float xf, float vf, float T);
    float getPosition(float t) const;
    float getVelocity(float t) const;
    float getAcceleration(float t) const;

private:
    float a0, a1, a2, a3;
};
