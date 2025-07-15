#include <Arduino.h>
#include "MotionController.hpp"

MotionController motion;

unsigned long lastUpdate = 0;
const unsigned long CONTROL_INTERVAL_MS = 25;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Starting MotionController test...");

  motion.begin();
  motion.moveForward(200.0, 1.2);  // Move 200 mm forward in 1.2 seconds
}

void loop() {
  unsigned long now = millis();
  if (now - lastUpdate >= CONTROL_INTERVAL_MS) {
    lastUpdate = now;

    motion.update(now);

    // Optional: print odometry or status info for debug
    Serial.print("X: "); Serial.print(motion.getX());  // You can add `getX()` to MotionController if needed
    Serial.print(" Finished: "); Serial.println(motion.isFinished());
  }
}
