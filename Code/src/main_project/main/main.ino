// ==== Autonomous main.ino: IMU yaw turn → encoder chain → LiDAR wall-follow ====

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "pin_config.hpp"
#include "robot_param.hpp"

#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "IMUOdometry.hpp"
#include "PIDController.hpp"
#include "Lidar.hpp"
#include "PositionController.hpp"

// --- Fallback gains if robot_param.hpp doesn't define them ---
#ifndef LEFT_POS_KP
#define LEFT_POS_KP 1.0f
#define LEFT_POS_KI 0.0f
#define LEFT_POS_KD 0.0f
#endif

// --- Objects ---
MotorController    motor;
EncoderOdometry    odom;
IMUOdometry        imu;
Lidar              lidar;
PositionController position(LEFT_POS_KP, LEFT_POS_KI, LEFT_POS_KD);

// ========== MAZE / DRIVE TUNING ==========
static const float CELL_SIZE_MM       = 175.0f;        // one cell
static const float TURN_RAD           = M_PI / 2.0f;   // 90 deg
static const float ANGLE_SCALE_RIGHT  = 0.84f;         // tune to hit exact 90
static const float ANGLE_SCALE_LEFT   = 0.79f;

static const int   PWM_DRIVE          = 150;           // forward speed
static const int   PWM_TURN           = 130;           // in-place turn speed
static const float LEFT_PWM_SCALE     = 1.00f;
static const float RIGHT_PWM_SCALE    = 0.975f;

// ========== IMU PID (yaw) ==========
static const float YAW_KP = 1.5f, YAW_KI = 0.1f, YAW_KD = 0.3f;
PIDController yawPID(YAW_KP, YAW_KI, YAW_KD);

static const int   MAX_PWM       = 200;
static const int   MIN_PWM_DEAD  = 15;
static const float DRIFT_DEG     = 0.0f;       // trigger recal at ~exact setpoint
static const float INIT_TOL      = 0.5f;       // coarse phase tolerance (deg)
static const int   INIT_PWM_SPD  = 30;

static float initialYawDeg = 0.0f, setpointYawDeg = 0.0f;

// ========== Wall-follow ==========
static const int   LIDAR_INDEX        = 1;       // adjust if your sensor index differs
static const float targetPositionSet  = 95.0f;   // mm from wall
static const unsigned long CONTROL_MS = 50;      // 20 Hz
static const size_t  LIDAR_AVG_COUNT  = 10;
static float         lidarBuffer[LIDAR_AVG_COUNT];
static size_t        lidarIndex = 0;
static float         lidarSum   = 0.0f;

// ========== Utils ==========
static float wrap180deg(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}
static float wrap180rad(float a) {
  while (a >  M_PI)   a -= 2.0f * M_PI;
  while (a <= -M_PI)  a += 2.0f * M_PI;
  return a;
}

// ========== Chain movement helpers (encoder odom) ==========
void forwardOneCell(int pwm = PWM_DRIVE) {
  odom.reset();
  int leftPWM  = (int)(pwm * LEFT_PWM_SCALE);
  int rightPWM = (int)(pwm * RIGHT_PWM_SCALE);
  motor.setMotorPWM(leftPWM, rightPWM);
  for (;;) {
    odom.update();
    float x = odom.getX(), y = odom.getY();
    if (sqrt(x*x + y*y) >= CELL_SIZE_MM) break;
    delay(2);
  }
  motor.setMotorPWM(0, 0);
}

void turnLeft(int pwm = PWM_TURN) {
  float target = TURN_RAD * ANGLE_SCALE_LEFT;
  odom.reset();
  int leftPWM  = (int)(-pwm * LEFT_PWM_SCALE);
  int rightPWM = (int)( pwm * RIGHT_PWM_SCALE);
  motor.setMotorPWM(leftPWM, rightPWM);
  for (;;) {
    odom.update();
    if (wrap180rad(odom.getTheta()) >= target) break;
    delay(2);
  }
  motor.setMotorPWM(0, 0);
}

void turnRight(int pwm = PWM_TURN) {
  float target = -TURN_RAD * ANGLE_SCALE_RIGHT;
  odom.reset();
  int leftPWM  = (int)( pwm * LEFT_PWM_SCALE);
  int rightPWM = (int)(-pwm * RIGHT_PWM_SCALE);
  motor.setMotorPWM(leftPWM, rightPWM);
  for (;;) {
    odom.update();
    if (wrap180rad(odom.getTheta()) <= target) break;
    delay(2);
  }
  motor.setMotorPWM(0, 0);
}

void executeCommands(const char *cmds) {
  for (int i = 0; cmds[i] != '\0'; i++) {
    char c = tolower(cmds[i]);
    switch (c) {
      case 'f': forwardOneCell(); break;
      case 'l': turnLeft();       break;
      case 'r': turnRight();      break;
      default: break;
    }
    delay(200);
  }
}

// ========== IMU yaw test (blocking) ==========
void runIMUTurnMinus90() {
  imu.update();
  initialYawDeg  = imu.getYawDegrees();
  setpointYawDeg = wrap180deg(initialYawDeg - 90.0f);

  yawPID.setGains(YAW_KP, YAW_KI, YAW_KD);
  yawPID.setOutputLimits(-MAX_PWM, MAX_PWM);
  yawPID.setDerivativeSmoothing(0.1f);
  yawPID.setUseDerivativeOnMeasurement(true);
  yawPID.reset(initialYawDeg);

  unsigned long lastT = millis();

  // Phase 1: coarse swing
  for (;;) {
    imu.update();
    float yaw = imu.getYawDegrees();
    float err = wrap180deg(setpointYawDeg - yaw);
    if (fabs(err) <= INIT_TOL) break;
    int dir = (err > 0) ? 1 : -1;
    motor.setMotorPWM(-dir * INIT_PWM_SPD, dir * INIT_PWM_SPD);
    delay(5);
  }
  motor.setMotorPWM(0, 0);
  delay(50);
  imu.update();
  yawPID.reset(imu.getYawDegrees());

  // Phase 2: fine PID settle (~1.5s)
  bool didRecal = false;
  for (unsigned long settleMs = 0; settleMs < 1500; ) {
    unsigned long now = millis();
    float dt = (now - lastT) * 0.001f;
    lastT = now;

    imu.update();
    float yaw   = imu.getYawDegrees();
    float error = wrap180deg(setpointYawDeg - yaw);
    float u     = yawPID.compute(error, yaw, dt);
    int   pwm   = (int)u;

    if      (pwm > 0 && pwm < MIN_PWM_DEAD)  pwm =  MIN_PWM_DEAD;
    else if (pwm < 0 && pwm > -MIN_PWM_DEAD) pwm = -MIN_PWM_DEAD;
    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

    motor.setMotorPWM(-pwm, +pwm);

    if (!didRecal && fabs(error) < DRIFT_DEG) {
      imu.begin(1, 0); delay(100); imu.update();
      yawPID.reset(imu.getYawDegrees());
      didRecal = true;
    }

    delay(10);
    settleMs += 10;
  }
  motor.setMotorPWM(0, 0);
}

// ========== Wall-follow helpers ==========
static float getSmoothedDistance(float newReading) {
  lidarSum -= lidarBuffer[lidarIndex];
  lidarBuffer[lidarIndex] = newReading;
  lidarSum += newReading;
  lidarIndex = (lidarIndex + 1) % LIDAR_AVG_COUNT;
  return lidarSum / (float)LIDAR_AVG_COUNT;
}

void wallFollowInit() {
  lidarSum = 0.0f; lidarIndex = 0;
  for (size_t i = 0; i < LIDAR_AVG_COUNT; i++) {
    lidarBuffer[i] = targetPositionSet;
    lidarSum += lidarBuffer[i];
  }
  position.setTarget(targetPositionSet);
}

// Run wall-follow for a fixed duration (ms), non-blocking timing inside
void wallFollowRunFor(unsigned long duration_ms) {
  unsigned long start = millis();
  unsigned long lastControl = start;

  while (millis() - start < duration_ms) {
    unsigned long now = millis();
    if (now - lastControl >= CONTROL_MS) {
      float dt = (now - lastControl) / 1000.0f;   // seconds
      lastControl = now;

      int raw = lidar.readDistance(LIDAR_INDEX);
      if (raw < 0) {
        // sensor error → creep forward to keep things gentle
        motor.setMotorPWM(50, 50);
      } else {
        float dist = getSmoothedDistance((float)raw);
        float control = position.update(dist, dt);
        int pwm = (int)constrain(control, -255, +255);

        // NOTE: sign may need flipping depending on sensor side
        motor.setMotorPWM(-pwm, -pwm);
      }
    }
    delay(1); // yield a little
  }

  motor.setMotorPWM(0, 0); // stop after the run
}

// ========== Arduino setup/loop ==========
void setup() {
  // Optional short settle so power rails are happy
  delay(750);

  motor.begin();
  odom.begin();
  imu.begin(1, 0);
  lidar.begin();

  // 1) IMU -90° turn first (as requested)
  runIMUTurnMinus90();

  // 2) Encoder odom chain movement to exercise left/right turns and distance
  executeCommands("lrlrl");

  // 3) Wall-follow for ~10 seconds (adjust as needed)
  wallFollowInit();
  wallFollowRunFor(10000);

  // End: ensure motors off
  motor.setMotorPWM(0, 0);
}

void loop() {
  // Nothing: the whole test runs in setup() and finishes.
}
