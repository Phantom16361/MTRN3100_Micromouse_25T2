// main.ino

#include <Arduino.h>
#include <math.h>

#include "EncoderOdometry.hpp"   // odometry from wheel ticks
#include "MotorController.hpp"   // DRV8835 interface
#include "PositionController.hpp"

// === Maze constants ===
static const float CELL_SIZE_MM = 180.0f;         // one maze cell
static const float TURN_RAD     = M_PI / 2.0f;    // 90° in radians

// === Tuning: compensate overshoot by cutting turns early ===
static const float ANGLE_SCALE = 0.90f;           // do 90% of the turn

// === Motor/drive parameters ===
static const int   PWM_DRIVE    = 150;            // forward speed
static const int   PWM_TURN     = 150;            // turn-in-place speed

// === Global objects ===
EncoderOdometry odom;
MotorController motor;

PositionController position(LEFT_POS_KP, LEFT_POS_KI, LEFT_POS_KD);
unsigned long lastControlTime = 0;


// Wrap angle into (–π, π]
static float wrap180rad(float a) {
  while (a >  M_PI)  a -= 2.0f * M_PI;
  while (a <= -M_PI) a += 2.0f * M_PI;
  return a;
}

/// Drive forward exactly one cell (180 mm)
void forwardOneCell() {
  odom.reset();
  position.reset();
  position.setTarget(CELL_SIZE_MM);

  unsigned long startTime = millis();  // Record the start time
  unsigned long lastControlTime = millis();  // For control interval timing

  while (millis() - startTime <= 4000) {  // Run for 10 seconds
    odom.update();

    unsigned long now = millis();
    if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
      float dt = (now - lastControlTime) / 1000.0;  // dt in seconds
      lastControlTime = now;

      int x = odom.getX();
      int output = static_cast<int>(position.update(static_cast<float>(x), dt));
      motor.setMotorPWM(output, output);
    }
  }
  motor.setMotorPWM(0, 0);
}

/// Turn in place 90° CCW (but only ANGLE_SCALE×90°)
void turnLeft() {
  float target = TURN_RAD * ANGLE_SCALE;
  odom.reset();
  motor.setMotorPWM(-0, +0);
  while (true) {
    odom.update();
    if (wrap180rad(odom.getTheta()) >= target) break;
    delay(2);
  }
  motor.setMotorPWM(0, 0);
}

/// Turn in place 90° CW (but only ANGLE_SCALE×90°)
void turnRight() {
  float target = -TURN_RAD * ANGLE_SCALE;
  odom.reset();
  motor.setMotorPWM(+0, -0);
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
      case 'f': forwardOneCell();  break;
      case 'l': turnLeft();        break;
      case 'r': turnRight();       break;
      default:  /* ignore */       break;
    }
    delay(200);  // brief pause between actions
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // give you 3 seconds to move away before we start driving
  Serial.println("Power on! Starting in 3 seconds...");
  delay(3000);

  odom.begin();   // attach interrupts, zero pose
  motor.begin();  // configure pins, stop motors

  Serial.println("Running command string: lfrfflfr");
  executeCommands("fffffff");
}

void loop() {
  // nothing here: all actions happen in setup()
}
