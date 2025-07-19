#ifndef IMU_ODOMETRY_HPP
#define IMU_ODOMETRY_HPP

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_light.hpp"

class IMUOdometry {
public:
    IMUOdometry();

    // Call in setup()
    void begin(uint8_t gyroCfg = 1, uint8_t accCfg = 0);

    // Call each loop() to update yaw
    void update();

    // Yaw [deg] and [rad]
    float getYawDegrees() const;
    float getYawRadians() const;

private:
    MPU6050 mpu;
    float   yawDeg;
};

#endif // IMU_ODOMETRY_HPP
