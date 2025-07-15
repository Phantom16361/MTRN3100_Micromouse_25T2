#include <Arduino.h>

// Core modules
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"

// Create odometry object using constants from robot_param.hpp
EncoderOdometry odom(WHEEL_RADIUS_MM, AXLE_LENGTH_MM, TICKS_PER_REV);

void setup() {
  Serial.begin(115200);
  delay(300);  // Give serial monitor time to connect

  Serial.println("Encoder Odometry Test Starting...");
  odom.begin();
}

void loop() {
  odom.update();

  // Print pose estimate
  Serial.print("Ticks L: ");
  Serial.print(odom.getLeftTicks());
  Serial.print(" | R: ");
  Serial.print(odom.getRightTicks());

  Serial.print(" || X: ");
  Serial.print(odom.getX(), 1);
  Serial.print(" mm | Y: ");
  Serial.print(odom.getY(), 1);
  Serial.print(" mm | θ: ");
  Serial.print(odom.getTheta(), 2);
  Serial.println(" rad");

  delay(100);  // 10 Hz update rate
}
