#pragma once

#include "Arduino.h"
#include "robot_param.hpp"
#include "pin_config.hpp"

class EncoderOdometry {
public:
    EncoderOdometry();

    void begin();
    void reset();
    void update();

    float getX() const;
    float getY() const;
    float getTheta() const;

    float getLeftSpeedMMs() const;
    float getRightSpeedMMs() const;

    long getLeftTicks() const;
    long getRightTicks() const;

    static void handleLeftA();
    static void handleRightA();

private:
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    long prevLeftTicks = 0;
    long prevRightTicks = 0;

    unsigned long lastUpdateTime = 0;

    static volatile long leftTicks;
    static volatile long rightTicks;
};
