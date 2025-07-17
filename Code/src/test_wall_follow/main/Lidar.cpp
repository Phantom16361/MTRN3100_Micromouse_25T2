#include "Lidar.hpp"

LidarModule::LidarModule() {}

void LidarModule::begin() {
    Wire.begin();

    // Setup enable pins
    for (int i = 0; i < 3; i++) {
        pinMode(enablePins[i], OUTPUT);
        digitalWrite(enablePins[i], LOW);  // All off initially
    }

    delay(100); // Allow sensors to power down

    // Initialize each LIDAR with unique I2C address
    for (int i = 0; i < 3; i++) {
        enableLidar(i);
        delay(10);
        lidars[i].init();
        lidars[i].setTimeout(200);
        lidars[i].setAddress(addresses[i]);
    }
}

int LidarModule::readDistance(LidarPosition pos) {
    if (pos < 0 || pos > 2) return -1;
    return lidars[pos].readRangeSingleMillimeters();
}

void LidarModule::enableLidar(int index) {
    disableAll();
    digitalWrite(enablePins[index], HIGH);
    delay(10); // Wait for boot
}

void LidarModule::disableAll() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(enablePins[i], LOW);
    }
}
