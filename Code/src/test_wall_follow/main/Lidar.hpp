#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <Wire.h>
#include <VL6180X.h>
#include "pin_config.hpp"

enum LidarPosition {
    LEFT = 0,
    FRONT = 1,
    RIGHT = 2
};

class Lidar {
public:
    Lidar();
    void begin();
    int readDistance(LidarPosition pos);  // returns distance in mm

private:
    VL6180X lidars[3];
    uint8_t enablePins[3] = {LIDAR_LEFT_EN, LIDAR_FRONT_EN, LIDAR_RIGHT_EN};
    uint8_t addresses[3] = {0x30, 0x31, 0x32}; // must be different!
    void enableLidar(int index);
    void disableAll();
};

#endif