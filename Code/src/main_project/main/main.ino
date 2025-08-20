// ==== main.ino: Forward-then-left-check with 8×8 bounds + dead-end safety ====
// Behavior (infinite):
//   0) DEAD-END SAFETY: If FRONT+LEFT+RIGHT are all walls, do a U-TURN (no forward).
//   1) Ensure the next forward step stays inside the 8×8 grid (rotate RIGHT until safe).
//   2) Drive forward exactly ONE cell with heading PI and front slow/stop gating.
//   3) Update (x,y,dir).
//   4) If there is NO LEFT wall and the left-adjacent cell is inside the grid, turn LEFT.
//   5) Repeat.

#include <Arduino.h>
#include <math.h>

#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "Lidar.hpp"

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

/* ========================= Maze / Drive Constants ========================= */

static const uint8_t MAZE_W = 8;
static const uint8_t MAZE_H = 8;

// Known start pose (grid coords + orientation).
// 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST
static const uint8_t START_X   = 0;
static const uint8_t START_Y   = 0;
static const uint8_t START_DIR = 0; // NORTH

// (Goal not used in this simple loop)
static const uint8_t GOAL_X    = 7;
static const uint8_t GOAL_Y    = 7;

static const float CELL_SIZE_MM       = 175.0f;   // one cell
static const float FORWARD_CELL_SCALE = 1.03f;    // small trim
static const float BRAKE_ZONE_MM      = 25.0f;    // taper near end

static const float TURN_RAD           = M_PI/2.0f; // 90°
static const int   PWM_DRIVE          = 150;
static const int   PWM_TURN           = 130;

/* ====================== Sensors / Timing ====================== */

static const int   MAX_VALID_MM       = 200;
static const int   SIDE_PRESENT_MM    = 140;      // side wall threshold
static const int   FRONT_SLOW_MM      = 170;      // start slowing down
static const int   FRONT_STOP_MM      = 110;      // treat as blocking wall
static const int   FRONT_MIN_BASE_PWM = 60;       // don't stall while slowing

static const uint32_t CTRL_DT_US      = 5000;     // 200 Hz ctrl
static const float    DT_SEC          = CTRL_DT_US * 1e-6f;

/* ====================== Heading hold (primary) ====================== */

static const float HEADING_KP_RAD     = 95.0f;    // ~1.7 PWM/deg
static const float HEADING_KI_RAD     = 35.0f;    // PWM/(rad·s)
static const int   HEADING_I_MAX      = 10;
static const int   HEADING_MAX_PWM    = 22;

/* ============================ Globals ============================ */

EncoderOdometry odom;
MotorController motor;
Lidar           lidar;

static inline int   clampInt(int v, int lo, int hi){ return v<lo?lo : v>hi?hi : v; }
static inline float clampF (float v, float lo, float hi){ return v<lo?lo : v>hi?hi : v; }
static float wrap180rad(float a){ while(a>M_PI) a-=2.f*M_PI; while(a<=-M_PI) a+=2.f*M_PI; return a; }

/* ========================== Grid pose helpers ========================== */

enum Dir : uint8_t {NORTH=0, EAST=1, SOUTH=2, WEST=3};
static const int8_t dx[4] = {0, 1, 0,-1};
static const int8_t dy[4] = {1, 0,-1, 0}; // y+ is "north/up" logically

static inline bool inBounds(int x,int y){ return x>=0 && y>=0 && x<MAZE_W && y<MAZE_H; }
static inline uint8_t leftOf (uint8_t d){ return (d+3) & 3; }
static inline uint8_t rightOf(uint8_t d){ return (d+1) & 3; }

/* ============================ Lidar helpers ============================ */

static int readMedian3(LidarPosition p){
  int a = lidar.readDistance(p);
  int b = lidar.readDistance(p);
  int c = lidar.readDistance(p);
  if (a<0) a=MAX_VALID_MM; if (b<0) b=MAX_VALID_MM; if (c<0) c=MAX_VALID_MM;
  int lo = min(a, min(b, c));
  int hi = max(a, max(b, c));
  return a + b + c - lo - hi; // median
}

static bool leftWallPresent (){
  int d = readMedian3(LEFT);
  return (d >= 0 && d <= SIDE_PRESENT_MM);
}
static bool rightWallPresent(){
  int d = readMedian3(RIGHT);
  return (d >= 0 && d <= SIDE_PRESENT_MM);
}
static bool frontWallPresent(){
  int d = readMedian3(FRONT);
  return (d >= 0 && d <= FRONT_STOP_MM); // treat as blocking if inside stop distance
}

/* ======================== Drive: forward one cell ======================== */

static void forwardOneCell(int pwm = PWM_DRIVE){
  const float target_mm = CELL_SIZE_MM * FORWARD_CELL_SCALE;
  odom.reset();

  float headI = 0.0f;
  float turn_cmd = 0.0f;
  uint32_t nextCtrl = micros();

  while (true){
    uint32_t now = micros();
    if ((int32_t)(now - nextCtrl) >= 0){
      // Base forward with gentle front gating
      int base = pwm;
      int dF = readMedian3(FRONT);
      if (dF <= FRONT_SLOW_MM){
        float frac = (float)(dF - FRONT_STOP_MM) / (float)(FRONT_SLOW_MM - FRONT_STOP_MM);
        float min_frac = (float)FRONT_MIN_BASE_PWM / (float)pwm;
        base = (int)(pwm * clampF(frac, min_frac, 1.0f));
        if (dF <= FRONT_STOP_MM) base = 0; // hard stop if too close
      }

      // Odometry distance & heading
      odom.update();
      float x = odom.getX(), y = odom.getY();
      float dist  = sqrtf(x*x + y*y);
      float theta = wrap180rad(odom.getTheta()); // +CCW from segment start

      // Taper near end of the cell
      if (target_mm - dist <= BRAKE_ZONE_MM){
        float frac = clampF((target_mm - dist) / BRAKE_ZONE_MM, 0.25f, 1.0f);
        base = (int)(base * frac);
      }

      // Heading PI: +theta (left) -> steer RIGHT (positive turn_cmd)
      float eHead = +theta;
      if (fabsf(eHead) < 0.35f){
        headI += HEADING_KI_RAD * eHead * DT_SEC;
        headI  = clampF(headI, -float(HEADING_I_MAX), +float(HEADING_I_MAX));
      } else {
        headI *= 0.85f; // leak when large error to avoid windup
      }
      int headPWM = (int)clampF(HEADING_KP_RAD * eHead + headI,
                                -float(HEADING_MAX_PWM), +float(HEADING_MAX_PWM));

      turn_cmd = (float)headPWM;

      // Apply to motors
      int leftPWM  = clampInt((int)(base + turn_cmd),  0, 255);
      int rightPWM = clampInt((int)(base - turn_cmd),  0, 255);
      motor.setMotorPWM(leftPWM, rightPWM);

      // Done?
      if (dist >= target_mm) break;
      nextCtrl += CTRL_DT_US;
    }
    delayMicroseconds(200);
  }

  motor.setMotorPWM(0,0);
}

/* ============================ Turn primitives ============================ */

static const float TURN_STOP_RAD = 0.03f;  // ~1.7°
static const float TURN_SLOW_RAD = 0.42f;  // taper near target
static const int   PWM_MIN_TURN  = 18;

static void turnLeft(int pwm_max = PWM_TURN){
  const float target = +TURN_RAD;
  odom.reset();
  for(;;){
    odom.update();
    float th = wrap180rad(odom.getTheta());
    float err= wrap180rad(target - th);
    if (fabsf(err) <= TURN_STOP_RAD) break;
    float scale = (fabsf(err) >= TURN_SLOW_RAD) ? 1.f : (fabsf(err)/TURN_SLOW_RAD);
    int pwm = PWM_MIN_TURN + (int)((pwm_max - PWM_MIN_TURN)*scale);
    int dir = (err > 0) ? +1 : -1;
    motor.setMotorPWM(-dir*pwm, +dir*pwm); // in-place
    delay(3);
  }
  motor.setMotorPWM(0,0);
}

static void turnRight(int pwm_max = PWM_TURN){
  const float target = -TURN_RAD;
  odom.reset();
  for(;;){
    odom.update();
    float th = wrap180rad(odom.getTheta());
    float err= wrap180rad(target - th);
    if (fabsf(err) <= TURN_STOP_RAD) break;
    float scale = (fabsf(err) >= TURN_SLOW_RAD) ? 1.f : (fabsf(err)/TURN_SLOW_RAD);
    int pwm = PWM_MIN_TURN + (int)((pwm_max - PWM_MIN_TURN)*scale);
    int dir = (err > 0) ? +1 : -1;
    motor.setMotorPWM(-dir*pwm, +dir*pwm); // in-place
    delay(3);
  }
  motor.setMotorPWM(0,0);
}

/* ========================== Bounds/Dead-end helpers ======================= */

// Rotate RIGHT until the next forward step would remain inside the 8×8 grid.
static void ensureNextForwardInside(uint8_t &dir, uint8_t x, uint8_t y){
  for (int k=0;k<4;k++){
    int nx = (int)x + dx[dir];
    int ny = (int)y + dy[dir];
    if (inBounds(nx,ny)) return;       // safe to go forward next
    // otherwise rotate right 90° and check again
    turnRight(PWM_TURN);
    dir = rightOf(dir);
  }
}

// Check for a dead-end (front, left, right all blocked). If so, U-TURN and return true.
static bool handleDeadEnd(uint8_t &dir){
  bool wF = frontWallPresent();
  bool wL = leftWallPresent();
  bool wR = rightWallPresent();
  if (wF && wL && wR){
    // U-turn (rotate 180°)
    turnRight(PWM_TURN);
    turnRight(PWM_TURN);
    dir = rightOf(rightOf(dir));
    return true;
  }
  return false;
}

/* ================================= Setup ================================= */

static uint8_t poseX = START_X, poseY = START_Y, poseDir = START_DIR;

void setup(){
  odom.begin();
  motor.begin();
  lidar.begin();
  delay(200); // small settle
}

/* ================================= Loop ================================== */

void loop(){
  // A) DEAD-END SAFETY: if all three sides are blocked, U-turn and skip forward
  if (handleDeadEnd(poseDir)){
    delay(60);
    return; // end this loop cycle; Arduino will call loop() again
  }

  // B) Make sure we won't step outside the 8×8 map before moving forward
  ensureNextForwardInside(poseDir, poseX, poseY);

  // C) Move forward exactly one cell
  forwardOneCell(PWM_DRIVE);

  // D) Update discrete pose on the grid
  poseX = (uint8_t)((int)poseX + dx[poseDir]);
  poseY = (uint8_t)((int)poseY + dy[poseDir]);

  // E) If no left wall AND left cell is in-bounds, turn left
  uint8_t leftDir = leftOf(poseDir);
  int     lx = (int)poseX + dx[leftDir];
  int     ly = (int)poseY + dy[leftDir];
  if (!leftWallPresent() && inBounds(lx, ly)){
    turnLeft(PWM_TURN);
    poseDir = leftDir;
  }

  // F) Repeat forever
  delay(60); // tiny settle
}
