#ifndef MPU6050_LIGHT_HPP
#define MPU6050_LIGHT_HPP

#include <Arduino.h>
#include <Wire.h>

// MPU-6050 I²C addresses & registers
#define MPU6050_ADDR                  0x68
#define MPU6050_SMPLRT_DIV_REGISTER   0x19
#define MPU6050_CONFIG_REGISTER       0x1A
#define MPU6050_GYRO_CONFIG_REGISTER  0x1B
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1C
#define MPU6050_PWR_MGMT_1_REGISTER   0x6B

#define MPU6050_ACCEL_OUT_REGISTER    0x3B
#define MPU6050_GYRO_OUT_REGISTER     0x43

#define RAD_2_DEG             57.29578f
#define CALIB_OFFSET_NB_MES   500
#define DEFAULT_GYRO_COEFF    0.98f

class MPU6050 {
public:
    // ctor: pass in your I2C bus (usually Wire)
    MPU6050(TwoWire &w);

    // Initialize; returns I2C status (0 = OK)
    byte begin(int gyro_config_num = 1, int acc_config_num = 0);

    // Read raw data and update fused angles
    void fetchData();
    void update();

    // Configuration & calibration
    byte setGyroConfig(int config_num);
    byte setAccConfig(int config_num);
    void setFilterGyroCoef(float gyro_coeff);
    void calcOffsets(bool is_calc_gyro = true, bool is_calc_acc = true);

    // Get fused Z-angle (yaw) in degrees
    float getAngleZ() const { return angleZ; }

private:
    TwoWire *wire;
    uint8_t  address;

    // Low-level I²C
    byte writeData(byte reg, byte data);
    byte readData(byte reg);

    // Sensor offsets & filter
    float gyro_lsb_to_degsec;
    float acc_lsb_to_g;
    float gyroXoffset, gyroYoffset, gyroZoffset;
    float accXoffset, accYoffset, accZoffset;
    float filterGyroCoef;

    // Raw readings
    float temp;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;

    // Fused angles
    float angleAccX, angleAccY;
    float angleX, angleY, angleZ;
    unsigned long preInterval;
};

#endif // MPU6050_LIGHT_HPP
