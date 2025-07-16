/**************************************************************
 *  File         : EncoderOdometry.hpp
 *  Author       : Jason E Tomczyk
 *  Description  : Header for encoder-based odometry class.
 *                 Provides position, heading, and velocity access
 *                 via interrupt-driven tick counting.
 * 
 *                 Do not edit unless discussed.
 *
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Declared update(), getX(), getTheta(), and speed
 *             estimators for left and right wheels.
 *************************************************************/

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
