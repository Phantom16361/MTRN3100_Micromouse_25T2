#include <Arduino.h>
#include <Wire.h>

#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PositionController.hpp"
#include "IMUOdometry.hpp"

// ———— Objects ————
EncoderOdometry    odom;
MotorController    motor;
PositionController position(LEFT_POS_KP, LEFT_POS_KI, LEFT_POS_KD);
IMUOdometry        imu;

// ———— Timing ————
unsigned long lastControlTime = 0;
const unsigned long CONTROL_MS = 25;

unsigned long lastImuTime = 0;
const unsigned long IMU_MS = 10;

// ———— Target ————
float targetPositionSet = 100.0f;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Wall follow with yaw");

  // IMU
  imu.begin(/*gyroCfg=*/1, /*accCfg=*/0);

  // Robot
  odom.begin();
  motor.begin();
  position.setTarget(targetPositionSet);
}

void loop() {
  // 1) IMU yaw
  imu.update();
  if (millis() - lastImuTime >= IMU_MS) {
    Serial.print("Yaw (°): ");
    Serial.println(imu.getYawDegrees(), 1);
    lastImuTime = millis();
  }

  // 2) Wall-follow every CONTROL_MS
  odom.update();
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_MS) {
    float dt = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;

    float x = odom.getX();
    int output = static_cast<int>(position.update(x, dt));
    motor.setMotorPWM(output, output);
  }
}
