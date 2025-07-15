#ifndef ENCODER_ODOMETRY_HPP
#define ENCODER_ODOMETRY_HPP

#include <Arduino.h>
#include "pin_config.hpp" // Contains encoder pin definitions

class EncoderOdometry {
public:
    EncoderOdometry(float wheelRadiusMM, float axleLengthMM, int ticksPerRevolution);

    void begin();
    void update(); // Call every loop
    void reset();

    // Position estimates (in mm and radians)
    float getX() const;
    float getY() const;
    float getTheta() const;

    // Velocity estimates
    float getLeftSpeedMMs() const;
    float getRightSpeedMMs() const;

    // Direct access to encoder counts
    long getLeftTicks() const;
    long getRightTicks() const;

private:
    // Physical robot parameters
    float radius;
    float axleLength;
    int ticksPerRev;
    float mmPerTick;

    // Pose estimate
    float x, y, theta;

    // Last update state
    long prevLeftTicks;
    long prevRightTicks;
    unsigned long lastUpdateTime;

    // Encoder tick counters
    static volatile long leftTicks;
    static volatile long rightTicks;

    // Interrupt handlers
    static void handleLeftA();
    static void handleRightA();
};

#endif // ENCODER_ODOMETRY_HPP
