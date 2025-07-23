// main.ino

#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PositionController.hpp"
#include "Lidar.hpp"

MotorController    motor;
Lidar              lidar;
PositionController position(LEFT_POS_KP, LEFT_POS_KI, LEFT_POS_KD);

static const float targetPositionSet     = 95.0f;
static const unsigned long CONTROL_MS = 50;  // e.g. 20 Hz control

// --- LIDAR averaging ---
static const size_t  LIDAR_AVG_COUNT = 10;
static float        lidarBuffer[LIDAR_AVG_COUNT];
static size_t       lidarIndex = 0;
static size_t       lidarCount = 0;
static float        lidarSum   = 0.0f;

unsigned long lastControlTime = 0;

void setup() {
  Serial.begin(9600);
  delay(3000);
  Serial.println("Wall follow with LIDAR smoothing");

  motor.begin();
  lidar.begin();
  position.setTarget(targetPositionSet);

  // init buffer
  for (size_t i = 0; i < LIDAR_AVG_COUNT; i++) {
    lidarBuffer[i] = targetPositionSet;
  }
  lidarCount = LIDAR_AVG_COUNT;
  for (size_t i = 0; i < lidarCount; i++) {
    lidarSum += lidarBuffer[i];
  }
  lastControlTime = millis();
}

float getSmoothedDistance(float newReading) {
  // remove oldest
  lidarSum -= lidarBuffer[lidarIndex];
  // push new
  lidarBuffer[lidarIndex] = newReading;
  lidarSum += newReading;
  // advance index
  lidarIndex = (lidarIndex + 1) % LIDAR_AVG_COUNT;
  // return average
  return lidarSum / (float)LIDAR_AVG_COUNT;
}

void loop() {
  unsigned long now = millis();
  
  if (now - lastControlTime < CONTROL_MS) return;
  float dt = (now - lastControlTime) / 2000.0f;
  lastControlTime = now;


  // raw LIDAR read
  float rawDist = lidar.readDistance(1);
  if (rawDist < 0) {
    // sensor error; back off or stop and skip this cycle
    Serial.println("LIDAR error, backing off");
    motor.setMotorPWM(50, 50);
    return;
  }

  // smooth it
  float dist = getSmoothedDistance(rawDist);

  Serial.print("Raw: ");
  Serial.print(rawDist, 1);
  Serial.print(" mm, Smoothed: ");
  Serial.print(dist, 1);
  Serial.println(" mm");

  // PID control
  float control = position.update(dist, dt);
  int pwm = (int)control;
  pwm = constrain(pwm, -255, +255);
  motor.setMotorPWM(-pwm, -pwm);
}
