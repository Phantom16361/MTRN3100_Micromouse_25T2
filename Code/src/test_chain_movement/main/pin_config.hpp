/**************************************************************
 *  File         : pin_config.hpp
 *  Author       : Jason E Tomczyk
 *  Description  : Centralized pin definitions for Micromouse hardware.
 *                 Maps symbolic names to physical GPIO pins for motors,
 *                 encoders, sensors, and display.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Declared symbolic macros for all relevant hardware pins
 *             including encoder channels and motor driver signals.
 *************************************************************/

#ifndef PIN_CONFIG_HPP
#define PIN_CONFIG_HPP

// ===================== Motor Driver Pins =====================
#define MOT1_PWM   11
#define MOT1_DIR   12
#define MOT2_PWM   9
#define MOT2_DIR   10

// ===================== Encoder Pins =====================
#define ENC1_A     2
#define ENC1_B     7
#define ENC2_A     3
#define ENC2_B     8

// ===================== I2C Pins (Arduino Nano) =====================
#define I2C_SDA    A4
#define I2C_SCL    A5

// ===================== OLED Display =====================
#define OLED_ADDR   0x3C
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_RESET  -1  // Not used on most I2C OLEDs

// ===================== Lidar Enable Pins =====================
#define LIDAR_LEFT_EN   A0
#define LIDAR_FRONT_EN  A1
#define LIDAR_RIGHT_EN  A2

#endif // PIN_CONFIG_HPP
