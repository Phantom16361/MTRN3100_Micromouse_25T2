// main.ino

#include <Arduino.h>
#include <EncoderOdometry.hpp>   // :contentReference[oaicite:0]{index=0}
#include <MotorController.hpp>    // :contentReference[oaicite:1]{index=1}
#include <math.h>

// === Maze / robot constants ===
const float CELL_SIZE_MM = 180.0f;       // one cell
const float TURN_RAD    = M_PI / 2.0f;   // 90°

EncoderOdometry odom;
MotorController motors;

// === Helpers ===
static float wrap180rad(float a) {
  while (a >  M_PI)  a -= 2.0f * M_PI;
  while (a <= -M_PI) a += 2.0f * M_PI;
  return a;
}

/// Drive straight until the robot has moved one cell (180 mm)
void forwardOneCell(int pwm = 150) {
  odom.reset();                // zero x,y,θ
  odom.update();
  motors.setMotorPWM(pwm, pwm);
  while (true) {
    odom.update();
    float dx = odom.getX();
    float dy = odom.getY();
    float dist = sqrt(dx*dx + dy*dy);
    if (dist >= CELL_SIZE_MM) break;
    // small delay to let interrupts fire:
    delay(2);
  }
  motors.setMotorPWM(0, 0);
}

/// Turn in place 90° CCW
void turnLeft(int pwm = 150) {
  odom.reset();
  odom.update();
  motors.setMotorPWM(-pwm, +pwm);
  while (true) {
    odom.update();
    float θ = wrap180rad(odom.getTheta());
    if (θ >= TURN_RAD) break;
    delay(2);
  }
  motors.setMotorPWM(0, 0);
}

/// Turn in place 90° CW
void turnRight(int pwm = 150) {
  odom.reset();
  odom.update();
  motors.setMotorPWM(+pwm, -pwm);
  while (true) {
    odom.update();
    float θ = wrap180rad(odom.getTheta());
    if (θ <= -TURN_RAD) break;
    delay(2);
  }
  motors.setMotorPWM(0, 0);
}

/// Parse & execute a command string: 'f' = forward, 'l' = left, 'r' = right
void executeCommands(const char *cmds) {
  for (int i = 0; cmds[i] != '\0'; i++) {
    switch (cmds[i]) {
      case 'f': forwardOneCell();  break;
      case 'l': turnLeft();        break;
      case 'r': turnRight();       break;
      default:  /* ignore */       break;
    }
    delay(200);  // brief pause between moves
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // init odometry & motors
  odom.begin();    // attach interrupts, zero state
  motors.begin();  // pinMode + stop

  // example run: start at (0,0,S), then lfrfflfr
  executeCommands("lfrfflfr");
}

void loop() {
  // nothing here—our run happens in setup()
}

// main.ino

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "IMUOdometry.hpp"

// === Maze & motion constants ===
static const float CELL_SIZE_MM = 180.0f;   // one cell length
static const float TURN_DEG     =  90.0f;   // degrees to turn
static const int   PWM_DRIVE    = 150;      // forward drive speed
static const int   PWM_TURN     = 150;      // spin‐in‐place speed

// global hardware objects
EncoderOdometry enc;
IMUOdometry      imu;
MotorController  motors;

// wrap degrees into (–180,180]
static float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

/// Move forward exactly one cell using encoder odometry
void forwardOneCell(int pwm = PWM_DRIVE) {
  enc.reset();
  motors.setMotorPWM(pwm, pwm);
  while (true) {
    enc.update();
    float dx = enc.getX();
    float dy = enc.getY();
    if (sqrt(dx*dx + dy*dy) >= CELL_SIZE_MM) break;
    delay(2);
  }
  motors.setMotorPWM(0, 0);
}

/// Turn 90° CCW using IMU yaw
void turnLeftIMU(int pwm = PWM_TURN) {
  // record starting yaw
  imu.update();
  float startYaw = imu.getYawDegrees();
  float target   = wrap180(startYaw + TURN_DEG);

  motors.setMotorPWM(-pwm, +pwm);
  while (true) {
    imu.update();
    float yaw = imu.getYawDegrees();
    float err = wrap180(target - yaw);
    // when err crosses zero from positive to ≤0, we've turned 90°
    if (err <= 0.0f) break;
    delay(2);
  }
  motors.setMotorPWM(0, 0);
}

/// Turn 90° CW using IMU yaw
void turnRightIMU(int pwm = PWM_TURN) {
  imu.update();
  float startYaw = imu.getYawDegrees();
  float target   = wrap180(startYaw - TURN_DEG);

  motors.setMotorPWM(+pwm, -pwm);
  while (true) {
    imu.update();
    float yaw = imu.getYawDegrees();
    float err = wrap180(target - yaw);
    // when err crosses zero from negative to ≥0, we've turned -90°
    if (err >= 0.0f) break;
    delay(2);
  }
  motors.setMotorPWM(0, 0);
}

/// Execute a string of moves: 'f' = forward, 'l' = left, 'r' = right
void executeCommands(const char *cmds) {
  for (int i = 0; cmds[i] != '\0'; i++) {
    char c = tolower(cmds[i]);
    switch (c) {
      case 'f': forwardOneCell();      break;
      case 'l': turnLeftIMU();         break;
      case 'r': turnRightIMU();        break;
      default:  /* skip invalid */     break;
    }
    delay(200); // pause between moves
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // init sensors & motors
  enc.begin();       // attach encoder interrupts, zero pose
  imu.begin(1, 0);   // init IMU + bias calibration
  delay(100);
  motors.begin();    // config motor pins & stop

  Serial.println("Executing: lfrfflfr");
  executeCommands("lfrfflfr");
}

void loop() {
  // nothing here—moves run once in setup()
}
