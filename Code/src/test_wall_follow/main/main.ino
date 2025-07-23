#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PositionController.hpp"
#include "Lidar.hpp"

MotorController motor;
Lidar lidar;

PositionController position(LEFT_POS_KP, LEFT_POS_KI, LEFT_POS_KD);   // Tweak gains
unsigned long lastControlTime = 0;
float targetPositionSet = 118.0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Wall follow");

  motor.begin();
  lidar.begin();
  position.setTarget(targetPositionSet);

}

void loop() {
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 2000.0;
    lastControlTime = now;

    float x = lidar.readDistance(1);
    Serial.print("Distance: ");
    Serial.print(x);
    Serial.println(" mm");

    if (x == -2) {
      motor.setMotorPWM(50, 50);
    } else {
      int output = static_cast<int>(position.update(x, dt));
      motor.setMotorPWM(-output, -output);
    }
    
    



  }
}
