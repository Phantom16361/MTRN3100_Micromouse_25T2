#include "IMUOdometry.hpp"

IMUOdometry::IMUOdometry()
  : mpu(Wire), yawDeg(0.0f)
{}

void IMUOdometry::begin(uint8_t gyroCfg, uint8_t accCfg) {
    Wire.begin();
    byte st = mpu.begin(gyroCfg, accCfg);
    if (st) {
      Serial.print("MPU init err: ");
      Serial.println(st);
      while (1);
    }
    mpu.calcOffsets();
}

void IMUOdometry::update() {
    mpu.update();
    yawDeg = mpu.getAngleZ();
    if (yawDeg > 180.0f)       yawDeg -= 360.0f;
    else if (yawDeg < -180.0f) yawDeg += 360.0f;
}

float IMUOdometry::getYawDegrees() const {
    return yawDeg;
}

float IMUOdometry::getYawRadians() const {
    return yawDeg * (PI / 180.0f);
}
