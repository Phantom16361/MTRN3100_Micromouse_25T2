#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PositionController.hpp"
#include "IMUOdometry.hpp"
#include "Lidar.hpp"
#include "PIDController.hpp"

MotorController motor;
IMUOdometry imu;
Lidar lidar;

// PID gains - define these in robot_param.hpp or here
#ifndef YAW_KP
#define YAW_KP 2.0f
#define YAW_KI 0.0f
#define YAW_KD 0.1f
#endif

PIDController yawPID(YAW_KP, YAW_KI, YAW_KD);

unsigned long lastControlTime = 0;
//const unsigned long CONTROL_INTERVAL_MS = 50; // 20 Hz control loop

float savedYaw = 0.0f;

struct LidarSnapshot {
  int left;
  int front;
  int right;
};

LidarSnapshot targetLidar;

enum TurnState {
  TURN_90_CW,
  WAIT_FOR_LIFT_CCW,
  CORRECT_CCW,
  WAIT_FOR_LIFT_CW,
  CORRECT_CW,
  DONE
};

TurnState state = TURN_90_CW;
bool wasLifted = false;

bool isYawAligned(float current, float target, float tol = 5.0f) {
  float err = target - current;
  while (err > 180.0f) err -= 360.0f;
  while (err < -180.0f) err += 360.0f;
  return abs(err) < tol;
}

LidarSnapshot getLidarSnapshot() {
  return { 
    lidar.readDistance(LEFT),
    lidar.readDistance(FRONT),
    lidar.readDistance(RIGHT)
  };
}

bool isLidarAligned(const LidarSnapshot& current, const LidarSnapshot& target, int tol = 30) {
  return abs(current.left  - target.left)  < tol &&
         abs(current.front - target.front) < tol &&
         abs(current.right - target.right) < tol;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("IMU + Lidar Turn Task Start");

  motor.begin();
  imu.begin();
  lidar.begin();
  imu.update();

  savedYaw = imu.getYawDegrees();

  yawPID.setTargetSetpoint(90.0f); // first turn target
  yawPID.reset();

  lastControlTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastControlTime < CONTROL_INTERVAL_MS) return;

  float dt = (now - lastControlTime) / 1000.0f;
  lastControlTime = now;

  imu.update();
  float yaw = imu.getYawDegrees();

  switch (state) {
  case TURN_90_CW: {
    float error = 90.0f - yaw;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    float control = yawPID.compute(error, dt, yaw);
    control = constrain(control, -100, 100);
    motor.setMotorPWM(control, -control);  // turn robot clockwise

    if (isYawAligned(yaw, 90.0f)) {
      motor.setMotorPWM(0, 0);
      savedYaw = yaw;  // save current yaw as setpoint
      state = WAIT_FOR_LIFT_CCW;
      Serial.println("Turn 90 CW done, waiting for lift CCW");
    }
    break;
  }

  case WAIT_FOR_LIFT_CCW: {
    LidarSnapshot current = getLidarSnapshot();
    if (!wasLifted && current.front > 200) { // lifted if lidar front sees no ground (distance jumps)
      wasLifted = true;
      targetLidar = current; // save snapshot
      state = CORRECT_CCW;
      yawPID.setTargetSetpoint(savedYaw - 120);  // target yaw after CCW rotate by demonstrator
      yawPID.reset();
      Serial.println("Lift detected, correcting CCW");
    }
    if (wasLifted && current.front < 150) {
      wasLifted = false; // lowered back down
    }
    break;
  }

  case CORRECT_CCW: {
    float targetYaw = savedYaw - 120;
    if (targetYaw < -180) targetYaw += 360;

    float error = targetYaw - yaw;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    float control = yawPID.compute(error, dt, yaw);
    control = constrain(control, -100, 100);
    motor.setMotorPWM(control, -control);  // rotate robot

    if (isYawAligned(yaw, targetYaw)) {
      motor.setMotorPWM(0, 0);
      savedYaw = yaw;
      state = WAIT_FOR_LIFT_CW;
      Serial.println("Corrected CCW, waiting for lift CW");
    }
    break;
  }

  case WAIT_FOR_LIFT_CW: {
    LidarSnapshot current = getLidarSnapshot();
    if (!wasLifted && current.front > 200) { // detect lift again
      wasLifted = true;
      targetLidar = current;
      state = CORRECT_CW;
      yawPID.setTargetSetpoint(savedYaw + 120);  // target yaw after CW rotate by demonstrator
      yawPID.reset();
      Serial.println("Lift detected, correcting CW");
    }
    if (wasLifted && current.front < 150) {
      wasLifted = false; // lowered back down
    }
    break;
  }

  case CORRECT_CW: {
    float targetYaw = savedYaw + 120;
    if (targetYaw > 180) targetYaw -= 360;

    float error = targetYaw - yaw;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    float control = yawPID.compute(error, dt, yaw);
    control = constrain(control, -100, 100);
    motor.setMotorPWM(control, -control);

    if (isYawAligned(yaw, targetYaw)) {
      motor.setMotorPWM(0, 0);
      state = DONE;
      Serial.println("Corrected CW, task complete");
    }
    break;
  }

  case DONE: {
    motor.setMotorPWM(0, 0);
    // Optionally stay idle here or do other stuff
    break;
  }
}
}
