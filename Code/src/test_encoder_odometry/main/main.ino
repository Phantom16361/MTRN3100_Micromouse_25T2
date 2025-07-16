/**************************************************************
 *  File         : main.ino
 *  Author       : Jason E Tomczyk
 *  Description  : Basic odometry test for encoders. Initializes
 *                 motor and odometry subsystems, approximates
 *                 current robot position and atitude based on encoder counts
 *                 relative to the initial start location.
 *                 Prints the referance frame coordinates to OLED.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Validated motor/encoder interaction in isolation.
 *             Prepared straight-line test suitable for Week 4
 *             barebones demonstration.
 *************************************************************/


#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"

#include "pin_config.hpp"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

EncoderOdometry odom;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Encoder Odometry Test Starting...");
  odom.begin();


  // Start of OLED Setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (true)
      ;  // halt
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Odometry OLED Ready");
  display.display();
  // End of OLED Setup
}

void loop() {
  odom.update();

  // Serial.print("Ticks L: ");
  // Serial.print(odom.getLeftTicks());
  // Serial.print(" | R: ");
  // Serial.print(odom.getRightTicks());
  // Serial.print(" || X: ");
  // Serial.print(odom.getX(), 1);
  // Serial.print(" mm | Y: ");
  // Serial.print(odom.getY(), 1);
  // Serial.print(" mm | θ: ");
  // Serial.print(odom.getTheta(), 2);
  // Serial.println(" rad");

  // Start of OLED print
  display.clearDisplay();
  display.setCursor(0, 0);

  // Position
  display.print("X: ");
  display.print(odom.getX(), 0);
  display.print("mm\nY: ");
  display.print(odom.getY(), 0);
  display.print("mm\nTh: ");
  display.print(odom.getTheta(), 2);
  display.print("rad");

  // Pulses
  display.setCursor(0, 40);  // Move down screen
  display.print("L: ");
  display.print(odom.getLeftTicks());
  display.print("  R: ");
  display.print(odom.getRightTicks());

  display.display();
}
