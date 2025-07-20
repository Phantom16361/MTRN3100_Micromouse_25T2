#include "MPU6050_light.hpp"
#include <Arduino.h>
#include <math.h>

static float wrap(float angle, float limit) {
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

MPU6050::MPU6050(TwoWire &w)
  : wire(&w),
    address(MPU6050_ADDR),
    gyro_lsb_to_degsec(0), acc_lsb_to_g(0),
    gyroXoffset(0), gyroYoffset(0), gyroZoffset(0),
    accXoffset(0), accYoffset(0), accZoffset(0),
    temp(0), accX(0), accY(0), accZ(0),
    gyroX(0), gyroY(0), gyroZ(0),
    angleAccX(0), angleAccY(0),
    angleX(0), angleY(0), angleZ(0),
    preInterval(0),
    filterGyroCoef(DEFAULT_GYRO_COEFF)
{}

byte MPU6050::begin(int gyro_cfg, int acc_cfg) {
  byte status = writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x01);
  writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_CONFIG_REGISTER,       0x00);
  setGyroConfig(gyro_cfg);
  setAccConfig(acc_cfg);
  update();
  preInterval = millis();
  return status;
}

byte MPU6050::writeData(byte reg, byte data) {
  wire->beginTransmission(address);
  wire->write(reg);
  wire->write(data);
  return wire->endTransmission();
}

byte MPU6050::readData(byte reg) {
  wire->beginTransmission(address);
  wire->write(reg);
  wire->endTransmission(false);
  wire->requestFrom(address, (uint8_t)1);
  return wire->read();
}

byte MPU6050::setGyroConfig(int cfg) {
  switch (cfg) {
    case 0: gyro_lsb_to_degsec = 131.0f; writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00); break;
    case 1: gyro_lsb_to_degsec =  65.5f; writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x08); break;
    case 2: gyro_lsb_to_degsec =  32.8f; writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x10); break;
    case 3: gyro_lsb_to_degsec =  16.4f; writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x18); break;
    default: gyro_lsb_to_degsec = 131.0f; writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00); break;
  }
  return 0;
}

byte MPU6050::setAccConfig(int cfg) {
  switch (cfg) {
    case 0: acc_lsb_to_g = 16384.0f; writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00); break;
    case 1: acc_lsb_to_g =  8192.0f; writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x08); break;
    case 2: acc_lsb_to_g =  4096.0f; writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x10); break;
    case 3: acc_lsb_to_g =  2048.0f; writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x18); break;
    default: acc_lsb_to_g = 16384.0f; writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00); break;
  }
  return 0;
}

void MPU6050::setFilterGyroCoef(float coef) {
  filterGyroCoef = (coef < 0 || coef > 1) ? DEFAULT_GYRO_COEFF : coef;
}

void MPU6050::calcOffsets(bool doG, bool doA) {
  if (doG) gyroXoffset = gyroYoffset = gyroZoffset = 0;
  if (doA) accXoffset = accYoffset = accZoffset = 0;
  float sumA[6] = {0};
  for (int i = 0; i < CALIB_OFFSET_NB_MES; i++) {
    fetchData();
    sumA[0] += accX; sumA[1] += accY; sumA[2] += (accZ - 1.0f);
    sumA[3] += gyroX; sumA[4] += gyroY; sumA[5] += gyroZ;
    delay(1);
  }
  if (doA) {
    accXoffset = sumA[0] / CALIB_OFFSET_NB_MES;
    accYoffset = sumA[1] / CALIB_OFFSET_NB_MES;
    accZoffset = sumA[2] / CALIB_OFFSET_NB_MES;
  }
  if (doG) {
    gyroXoffset = sumA[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = sumA[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = sumA[5] / CALIB_OFFSET_NB_MES;
  }
}

void MPU6050::fetchData() {
  wire->beginTransmission(address);
  wire->write(MPU6050_ACCEL_OUT_REGISTER);
  wire->endTransmission(false);
  wire->requestFrom(address, (uint8_t)14);

  int16_t raw[7];
  for (int i = 0; i < 7; i++) {
    raw[i] = (wire->read() << 8) | wire->read();
  }

  accX  = raw[0] / acc_lsb_to_g  - accXoffset;
  accY  = raw[1] / acc_lsb_to_g  - accYoffset;
  accZ  = raw[2] / acc_lsb_to_g  - accZoffset;
  gyroX = raw[4] / gyro_lsb_to_degsec - gyroXoffset;
  gyroY = raw[5] / gyro_lsb_to_degsec - gyroYoffset;
  gyroZ = raw[6] / gyro_lsb_to_degsec - gyroZoffset;
}

void MPU6050::update() {
  fetchData();
  float sgZ = accZ < 0 ? -1 : 1;
  angleAccX = atan2(accY, sgZ * sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG;
  angleAccY = -atan2(accX, sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG;

  unsigned long now = millis();
  float dt = (now - preInterval) * 1e-3f;
  preInterval = now;

  angleX += gyroX * dt;
  angleY += gyroY * dt;
  angleZ += gyroZ * dt;
  // optionally combine with acc-based tilt on X/Y here using wrap/filter...
}