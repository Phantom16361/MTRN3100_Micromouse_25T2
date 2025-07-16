#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PositionController.hpp"

EncoderOdometry odom;
MotorController motor;

PositionController position(LEFT_POS_KP, LEFT_POS_KI, LEFT_POS_KD);   // Tweak gains


unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL_MS = 25;

float targetPositionSet = 100.0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Wall follow");

  odom.begin();
  motor.begin();
  position.setTarget(targetPositionSet);

}

void loop() {
  odom.update();

  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 2000.0;
    lastControlTime = now;

    float x = odom.getX();
    
    int output = static_cast<int>(position.update(x, dt));

    motor.setMotorPWM(output, output);

  }
}
