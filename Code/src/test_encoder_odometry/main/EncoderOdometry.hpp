#ifndef ENCODER_ODOMETRY_HPP
#define ENCODER_ODOMETRY_HPP

#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"

class EncoderOdometry {
public:
    EncoderOdometry(float wheelRadiusMM, float axleLengthMM, int ticksPerRevolution);

    void begin();
    void update();
    void reset();

    float getX() const;
    float getY() const;
    float getTheta() const;

    float getLeftSpeedMMs() const;
    float getRightSpeedMMs() const;

    long getLeftTicks() const;
    long getRightTicks() const;

private:
    float radius;
    float axleLength;
    int ticksPerRev;
    float mmPerTick;

    float x, y, theta;

    long prevLeftTicks;
    long prevRightTicks;
    unsigned long lastUpdateTime;

    static volatile long leftTicks;
    static volatile long rightTicks;

    static void handleLeftA();
    static void handleRightA();
};

#endif // ENCODER_ODOMETRY_HPP
