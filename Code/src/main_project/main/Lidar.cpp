#include <Wire.h>
#include "Lidar.hpp"

Lidar::Lidar() {}

void Lidar::begin() {
    Wire.begin();

    for (int i = 0; i < 3; ++i) {
        pinMode(enablePins[i], OUTPUT);
        digitalWrite(enablePins[i], LOW);
    }

    delay(10);

    for (int i = 0; i < 3; ++i) {
        enableLidar(i);
        delay(50);

        lidars[i].init();
        lidars[i].configureDefault();
        lidars[i].setTimeout(250);
        lidars[i].setAddress(addresses[i]);
        delay(10);
    }
}

int Lidar::readDistance(LidarPosition pos) {
    if (pos < 0 || pos > 2) return -1;
    int distance = lidars[pos].readRangeSingleMillimeters();
    if (distance == 255) return -2;  // keep your sentinel
    return distance;
}

void Lidar::enableLidar(int index) { digitalWrite(enablePins[index], HIGH); }
void Lidar::disableAll() { for (int i = 0; i < 3; ++i) digitalWrite(enablePins[i], LOW); }
