// main.ino

#include <Arduino.h>
#include <math.h>

#include "EncoderOdometry.hpp"   // odometry from wheel ticks
#include "MotorController.hpp"   // DRV8835 interface

// === Maze constants ===
static const float CELL_SIZE_MM     = 175.0f;    // one maze cell
static const float TURN_RAD         = M_PI / 2.0f; // 90° in radians

// === Turn tuning ===
// // FOR ACTUAL SIMIULATION
//static const float ANGLE_SCALE_RIGHT      = 0.91f;
// static const float ANGLE_SCALE_LEFT       = 0.86f;

static const float ANGLE_SCALE_RIGHT      = 0.84f;
static const float ANGLE_SCALE_LEFT       = 0.79f;

// === Motor/drive parameters ===
static const int   PWM_DRIVE        = 150;      // forward speed
static const int   PWM_TURN         = 130;      // turn-in-place speed

// FOR ACTUAL SIMIULATION
// static const float LEFT_PWM_SCALE   = 1.00f;    // scale for left motor PWM
// static const float RIGHT_PWM_SCALE  = 0.975f;    // scale for right motor PWM

static const float LEFT_PWM_SCALE   = 1.00f;    // scale for left motor PWM
static const float RIGHT_PWM_SCALE  = 0.975f;    // scale for right motor PWM

// === Global objects ===
EncoderOdometry odom;
MotorController motor;

// Wrap angle into (–π, π]
static float wrap180rad(float a) {
  while (a >  M_PI)   a -= 2.0f * M_PI;
  while (a <= -M_PI)  a += 2.0f * M_PI;
  return a;
}

/// Drive forward exactly one cell (180 mm)
void forwardOneCell(int pwm = PWM_DRIVE) {
  odom.reset();
  int leftPWM  = (int)(pwm * LEFT_PWM_SCALE);
  int rightPWM = (int)(pwm * RIGHT_PWM_SCALE);
  motor.setMotorPWM(leftPWM, rightPWM);
  while (true) {
    odom.update();
    float x = odom.getX();
    float y = odom.getY();
    if (sqrt(x*x + y*y) >= CELL_SIZE_MM) break;
    delay(2);
  }
  motor.setMotorPWM(0, 0);
}

/// Turn in place 90° CCW
void turnLeft(int pwm = PWM_TURN) {
  float target  = TURN_RAD * ANGLE_SCALE_LEFT;
  odom.reset();
  int leftPWM   = (int)(-pwm * LEFT_PWM_SCALE);
  int rightPWM  = (int)(pwm * RIGHT_PWM_SCALE);
  motor.setMotorPWM(leftPWM, rightPWM);
  while (true) {
    odom.update();
    if (wrap180rad(odom.getTheta()) >= target) break;
    delay(2);
  }
  motor.setMotorPWM(0, 0);
}

/// Turn in place 90° CW
void turnRight(int pwm = PWM_TURN) {
  float target  = -TURN_RAD * ANGLE_SCALE_RIGHT;
  odom.reset();
  int leftPWM   = (int)(pwm * LEFT_PWM_SCALE);
  int rightPWM  = (int)(-pwm * RIGHT_PWM_SCALE);
  motor.setMotorPWM(leftPWM, rightPWM);
  while (true) {
    odom.update();
    if (wrap180rad(odom.getTheta()) <= target) break;
    delay(2);
  }
  motor.setMotorPWM(0, 0);
}

/// Execute a null-terminated string of 'f','l','r' commands
void executeCommands(const char *cmds) {
  for (int i = 0; cmds[i] != '\0'; i++) {
    char c = tolower(cmds[i]);
    switch (c) {
      case 'f': forwardOneCell(); break;
      case 'l': turnLeft();       break;
      case 'r': turnRight();      break;
      default:  /* ignore */      break;
    }
    delay(200);  // brief pause between actions
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Waiting 3s before start...");
  delay(3000);

  odom.begin();   // attach interrupts, zero pose
  motor.begin();  // configure pins, stop motors

  Serial.println("Running command sequence: rlrlrlrlrlff");
  executeCommands("rffff");
}

void loop() {
  // nothing here: all actions happen in setup()
}
