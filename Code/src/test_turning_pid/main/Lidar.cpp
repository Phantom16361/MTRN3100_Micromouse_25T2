#include "Lidar.hpp"

Lidar::Lidar() {}

void Lidar::begin() {
    Wire.begin();

    // Setup enable pins and disable all sensors
    for (int i = 0; i < 3; ++i) {
        pinMode(enablePins[i], OUTPUT);
        digitalWrite(enablePins[i], LOW);  // Ensure all sensors are off
    }

    delay(10);  // Small delay to ensure shutdown

    for (int i = 0; i < 3; ++i) {
        enableLidar(i);
        delay(50);  // Allow sensor to power up

        lidars[i].init();
        lidars[i].configureDefault();
        lidars[i].setTimeout(250);
        lidars[i].setAddress(addresses[i]);  // Assign unique I2C address
        delay(10);  // Let I2C settle before next sensor is enabled
    }
}

int Lidar::readDistance(LidarPosition pos) {
    if (pos < 0 || pos > 2) return -5;
    return lidars[pos].readRangeSingleMillimeters();
}

void Lidar::enableLidar(int index) {
    digitalWrite(enablePins[index], HIGH);
}

void Lidar::disableAll() {
    for (int i = 0; i < 3; ++i) {
        digitalWrite(enablePins[i], LOW);
    }
}