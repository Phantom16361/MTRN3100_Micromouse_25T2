#include "Lidar.hpp"

Lidar::Lidar() {}

void Lidar::begin() {
    Wire.begin();

    // Setup enable pins and power down all sensors
    for (int i = 0; i < 3; i++) {
        pinMode(enablePins[i], OUTPUT);
        digitalWrite(enablePins[i], LOW);
    }

    delay(100); // Ensure all sensors are off

    // Power and initialize each sensor one at a time
    for (int i = 0; i < 3; i++) {
        enableLidar(i);
        delay(50); // Allow time for the sensor to boot

        lidars[i].init();
        lidars[i].configureDefault();
        lidars[i].setTimeout(500);
        lidars[i].setAddress(addresses[i]); // Assign new unique address
        disableAll(); // Turn off before moving to next
    }

    // Reactivate all sensors at their new addresses
    for (int i = 0; i < 3; i++) {
        digitalWrite(enablePins[i], HIGH);
        delay(10);
    }
}

int Lidar::readDistance(LidarPosition pos) {
    if (pos < 0 || pos > 2) return -1;
    return lidars[pos].readRangeSingleMillimeters();
}

void Lidar::enableLidar(int index) {
    disableAll();
    digitalWrite(enablePins[index], HIGH);
    delay(10); // Wait for boot-up
}

void Lidar::disableAll() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(enablePins[i], LOW);
    }
}
