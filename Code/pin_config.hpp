#ifndef PIN_CONFIG_HPP
#define PIN_CONFIG_HPP

// DRV8835 Motor Driver Pins
#define MOT1_PWM  11
#define MOT1_DIR  12
#define MOT2_PWM  9
#define MOT2_DIR  10

// Encoder Pins
#define ENC1_A    2
#define ENC1_B    7
#define ENC2_A    3
#define ENC2_B    8

// I2C Pins (Arduino Nano - fixed)
#define I2C_SDA   A4
#define I2C_SCL   A5

// Lidar Enable Pins
#define LIDAR_LEFT_EN   A0
#define LIDAR_FRONT_EN  A1
#define LIDAR_RIGHT_EN  A2  // Corrected: sensor3 = right lidar

// I2C Addresses (defaults — can be changed in code with .setAddress)
#define LIDAR_DEFAULT_ADDR  0x29
#define OLED_ADDR           0x3C

#endif // PIN_CONFIG_HPP
