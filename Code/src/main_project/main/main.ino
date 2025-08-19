// ==== main.ino: Forward-one-cell (VL6180X L/F/R) + encoder 90° turns ====
// Goal: go straight. Heading PI is primary; walls apply gentle nudges only.
// - Remembers corridor offset at cell entry and maintains it.
// - Large deadband + confirmation + LPF + slew for calm behavior.
// - Adaptive blend: more wall influence when near a wall or when off-center.
// - Single-wall standoff so it corrects away when only one wall is seen.
// - No IMU. No motor PWM trims.

#include <Arduino.h>
#include <math.h>
#include <ctype.h>

#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "Lidar.hpp"

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

/* ========================= Maze / Drive Constants ========================= */

static const float CELL_SIZE_MM         = 175.0f;   // one maze cell
static const float FORWARD_CELL_SCALE   = 1.03f;    // fine-tune forward distance
static const float BRAKE_ZONE_MM        = 25.0f;    // taper near end

static const float TURN_RAD             = M_PI / 2.0f; // 90°

// 90° turn scalars
static const float ANGLE_SCALE_LEFT     = 1.00f;
static const float ANGLE_SCALE_RIGHT    = 1.00f;

// Speeds
static const int   PWM_DRIVE            = 150;
static const int   PWM_TURN             = 130;

// Motor calibration (no trim)
static const float LEFT_PWM_SCALE       = 1.00f;
static const float RIGHT_PWM_SCALE      = 1.00f;

/* ====================== Sensors / Timing ====================== */

static const int   MAX_VALID_MM         = 200;

static const uint32_t CTRL_DT_US        = 5000;    // 200 Hz control
static const uint32_t LIDAR_DT_US       = 20000;   // ~50 Hz per sensor
static const float    DT_SEC            = CTRL_DT_US * 1e-6f;

static const float SIDE_ALPHA           = 0.50f;   // EMA for raw distance
static const uint32_t HOLD_INVALID_US   = 100000;  // keep last valid 100 ms

/* ====================== Corridor logic (gentle) ====================== */

// A side is "present" only if closer than this
static const int   SIDE_PRESENT_MM      = 140;

// Push-away if really close (cap the push)
static const int   TOO_CLOSE_MM         = 40;
static const int   REPULSE_MAX_PWM      = 14;

// Target lateral balance: use corridor difference (dR - dL)
static const int   CENTER_DEADBAND_MM   = 14;      // big deadband
static const int   CENTER_CONFIRM_CYCLES= 4;       // must persist
static const uint32_t FRESH_WINDOW_US   = 50000;   // both sides fresh

// Very light wall centering gain (mm -> PWM)
static const float SIDE_KP              = 0.10f;   // small on purpose
static const float CORR_LPF_ALPHA       = 0.20f;   // smooth target
static const int   STEER_MAX_PWM        = 20;      // absolute cap
static const float CORR_FRAC_OF_BASE    = 0.20f;   // also limit vs base
static const int   TURN_SLEW_PER_CYCLE  = 3;       // slow ramp
static const int   SIDE_SLOW_BASE       = 90;      // optional base cap on repulsion

// === NEW: Single-wall standoff control ===
static const int   SINGLE_NEAR_MM       = 90;      // start correcting if closer than this
static const int   SINGLE_TARGET_MM     = 70;      // desired distance from that wall
static const int   SINGLE_DEADBAND_MM   = 6;
static const float SINGLE_KP            = 0.14f;   // single-wall proportional gain

/* ====================== Heading hold (primary) ====================== */
// Encoder-only PI that keeps heading ~0 during the cell
static const float HEADING_KP_RAD       = 95.0f;   // ~1.7 PWM/deg
static const float HEADING_KI_RAD       = 35.0f;   // PWM/(rad·s)
static const int   HEADING_I_MAX        = 10;      // integral clamp
static const int   HEADING_MAX_PWM      = 22;      // absolute cap

/* ====================== Front gating (gentle) ====================== */
#define  FRONT_SLOW_ENABLE   1
#define  FRONT_STOP_ENABLE   0
static const int   FRONT_SLOW_MM        = 170;
static const int   FRONT_STOP_MM        = 110;
static const int   FRONT_MIN_BASE_PWM   = 60;      // don’t stall while slowing

/* ============================ Globals ============================ */

EncoderOdometry odom;
MotorController motor;
Lidar           lidar;

static inline int   clampInt(int v, int lo, int hi){ return v<lo?lo : v>hi?hi : v; }
static inline float clampF (float v, float lo, float hi){ return v<lo?lo : v>hi?hi : v; }
static float wrap180rad(float a){ while(a>M_PI) a-=2.f*M_PI; while(a<=-M_PI) a+=2.f*M_PI; return a; }

struct SideState {
  float    mm = MAX_VALID_MM;
  bool     valid = false;
  uint32_t t_last_valid = 0;
};
static SideState side[3]; // 0=LEFT, 1=FRONT, 2=RIGHT

static void update_one_lidar(int which){
  LidarPosition pos = (which==0)?LEFT:(which==1)?FRONT:RIGHT;
  int raw = lidar.readDistance(pos);
  uint32_t now = micros();
  if (raw>=0 && raw<=MAX_VALID_MM){
    side[which].mm += SIDE_ALPHA * (raw - side[which].mm);
    side[which].valid = true;
    side[which].t_last_valid = now;
  } else {
    side[which].valid = false;
    if (now - side[which].t_last_valid > HOLD_INVALID_US){
      side[which].mm += 0.15f * (MAX_VALID_MM - side[which].mm);
    }
  }
}

/* ========================= Forward One Cell ========================= */

void forwardOneCell(int pwm = PWM_DRIVE){
  const float target_mm = CELL_SIZE_MM * FORWARD_CELL_SCALE;
  odom.reset();

  // Prime smoothing
  for (int i=0;i<3;i++){
    LidarPosition pos = (i==0)?LEFT:(i==1)?FRONT:RIGHT;
    int r = lidar.readDistance(pos);
    side[i].mm    = (r>=0 && r<=MAX_VALID_MM) ? (float)r : (float)MAX_VALID_MM;
    side[i].valid = (r>=0 && r<=MAX_VALID_MM);
    side[i].t_last_valid = micros();
  }

  // Corridor baseline (remember offset at entry so we don’t chase a wall)
  float baselineDiff = 0.0f; // desired (dR - dL)
  {
    bool leftPresent  = (side[0].valid && side[0].mm < SIDE_PRESENT_MM);
    bool rightPresent = (side[2].valid && side[2].mm < SIDE_PRESENT_MM);
    if (leftPresent && rightPresent) {
      baselineDiff = side[2].mm - side[0].mm; // keep this difference through the cell
    } else {
      baselineDiff = 0.0f;
    }
  }

  uint32_t t_next_ctrl  = micros();
  uint32_t t_next_lidar = micros();
  int rr = 0;

  float corrFilt = 0.0f;      // filtered correction target (PWM)
  float turn_cmd = 0.0f;      // post-slew PWM
  int   persist  = 0;         // confirmation counter

  // Heading PI
  float headI = 0.0f;

  while (true){
    uint32_t now = micros();

    // Stagger sensor reads
    if ((int32_t)(now - t_next_lidar) >= 0) {
      update_one_lidar(rr);
      rr = (rr + 1) % 3;
      t_next_lidar += LIDAR_DT_US;
    }

    if ((int32_t)(now - t_next_ctrl) >= 0) {
      float dL = side[0].mm, dF = side[1].mm, dR = side[2].mm;
      bool  vL = side[0].valid || (now - side[0].t_last_valid <= HOLD_INVALID_US);
      bool  vF = side[1].valid || (now - side[1].t_last_valid <= HOLD_INVALID_US);
      bool  vR = side[2].valid || (now - side[2].t_last_valid <= HOLD_INVALID_US);

      // Base forward speed with gentle front slow-down
      int base = pwm;
      #if FRONT_SLOW_ENABLE
        if (vF && dF < FRONT_SLOW_MM){
          float frac = (dF - (float)FRONT_STOP_MM) / (float)(FRONT_SLOW_MM - FRONT_STOP_MM);
          float min_frac = FRONT_STOP_ENABLE ? 0.0f : min((float)FRONT_MIN_BASE_PWM/(float)pwm, 1.0f);
          frac = clampF(frac, min_frac, 1.0f);
          base = (int)(pwm * frac);
        }
      #endif
      #if FRONT_STOP_ENABLE
        if (vF && dF <= FRONT_STOP_MM){ base = 0; turn_cmd = 0; }
      #endif

      // Distance & heading
      odom.update();
      float x = odom.getX(), y = odom.getY();
      float dist  = sqrtf(x*x + y*y);
      float theta = wrap180rad(odom.getTheta()); // +CCW from this segment start

      // Taper near the cell end
      if (target_mm - dist <= BRAKE_ZONE_MM){
        float frac = clampF((target_mm - dist) / BRAKE_ZONE_MM, 0.25f, 1.0f);
        base = (int)(base * frac);
      }

      // --- Heading PI (primary) ---
      // Positive theta (CCW/left) must steer RIGHT (positive turn_cmd)
      float eHead = +theta;  // want heading ~0 through the cell
      // Integrate softly; leak when large or near walls to avoid fighting wall terms
      bool nearAnyWall = (vL && dL < SINGLE_NEAR_MM) || (vR && dR < SINGLE_NEAR_MM);
      if (!nearAnyWall && fabsf(eHead) < 0.35f) {
        headI += HEADING_KI_RAD * eHead * DT_SEC;
        headI = clampF(headI, -float(HEADING_I_MAX), +float(HEADING_I_MAX));
      } else {
        headI *= 0.85f; // leak integral near walls or large heading error
      }
      int headPWM = (int)clampF(HEADING_KP_RAD * eHead + headI,
                                -float(HEADING_MAX_PWM), +float(HEADING_MAX_PWM));

      // --- Wall correction (secondary) ---
      const bool leftFresh  = (now - side[0].t_last_valid) <= FRESH_WINDOW_US;
      const bool rightFresh = (now - side[2].t_last_valid) <= FRESH_WINDOW_US;

      const bool leftPresent  = vL && (dL < SIDE_PRESENT_MM)  && leftFresh;
      const bool rightPresent = vR && (dR < SIDE_PRESENT_MM)  && rightFresh;

      int wallPWM = 0;

      // 1) Repulsion if truly close (tiny cap) — strongest priority
      const bool leftClose  = leftPresent  && (dL <= TOO_CLOSE_MM);
      const bool rightClose = rightPresent && (dR <= TOO_CLOSE_MM);
      if (leftClose ^ rightClose){
        wallPWM = leftClose ? +min(REPULSE_MAX_PWM, (int)(TOO_CLOSE_MM - dL))
                            : -min(REPULSE_MAX_PWM, (int)(TOO_CLOSE_MM - dR));
        base = min(base, SIDE_SLOW_BASE);
        persist = 0;
      }
      // 2) Both walls present: center about baseline difference (calm + confirmed)
      else if (leftPresent && rightPresent) {
        float diff = (dR - dL) - baselineDiff; // + => closer to left than baseline
        if (fabsf(diff) > (float)CENTER_DEADBAND_MM) {
          if (++persist >= CENTER_CONFIRM_CYCLES) {
            wallPWM = (int)(SIDE_KP * diff);  // small & calm
          }
        } else {
          persist = 0;
        }
      }
      // 3) Single wall present: hold a gentle standoff if you're getting close
      else if (leftPresent ^ rightPresent) {
        persist = 0;
        if (leftPresent && dL < SINGLE_NEAR_MM) {
          float e = (float)SINGLE_TARGET_MM - dL; // + if too close to left
          if (fabsf(e) > (float)SINGLE_DEADBAND_MM) {
            wallPWM = (int)(SINGLE_KP * e); // + => steer right (away from left)
          }
        } else if (rightPresent && dR < SINGLE_NEAR_MM) {
          float e = dR - (float)SINGLE_TARGET_MM; // - if too close to right
          if (fabsf(e) > (float)SINGLE_DEADBAND_MM) {
            wallPWM = (int)(SINGLE_KP * e); // - => steer left (away from right)
          }
        }
      } else {
        // No reliable walls -> rely on heading only
        persist = 0;
      }

      // --- Adaptive blending (how much we trust walls vs heading) ---
      // Start mostly heading; increase wall weight when needed.
      float wHead = 0.80f; // default: lean on heading
      bool bothWalls = leftPresent && rightPresent;
      if (bothWalls) {
        wHead = 0.75f;
        float diff = fabsf((dR - dL) - baselineDiff);
        if (diff > (float)CENTER_DEADBAND_MM) wHead = 0.60f;   // off-center -> more wall help
      }
      if (leftClose || rightClose)       wHead = 0.35f;        // very close -> trust walls more
      else if ((leftPresent && dL < SINGLE_NEAR_MM) ||
               (rightPresent && dR < SINGLE_NEAR_MM)) wHead = 0.55f; // single-wall near

      // Combine
      float corrTarget = wHead * (float)headPWM + (1.0f - wHead) * (float)wallPWM;

      // LPF and clamp (also bound to fraction of current base)
      corrFilt += CORR_LPF_ALPHA * (corrTarget - corrFilt);
      int corrLimit = min(STEER_MAX_PWM, (int)(base * CORR_FRAC_OF_BASE));
      int turn_target = clampInt((int)lrintf(corrFilt), -corrLimit, +corrLimit);

      // Guardrail — never steer TOWARD a close/near wall
      if (bothWalls) {
        if (dL + 8 < dR && turn_target < 0) turn_target = 0; // nearer LEFT, forbid left steer
        if (dR + 8 < dL && turn_target > 0) turn_target = 0; // nearer RIGHT, forbid right steer
      } else {
        if (leftPresent  && dL < SINGLE_NEAR_MM && turn_target < 0) turn_target = 0;
        if (rightPresent && dR < SINGLE_NEAR_MM && turn_target > 0) turn_target = 0;
      }

      // Slew-limit
      int dturn = turn_target - (int)turn_cmd;
      if (dturn >  TURN_SLEW_PER_CYCLE) dturn =  TURN_SLEW_PER_CYCLE;
      if (dturn < -TURN_SLEW_PER_CYCLE) dturn = -TURN_SLEW_PER_CYCLE;
      turn_cmd += dturn;

      // +turn_cmd => steer RIGHT (left faster, right slower)
      int leftPWM  = clampInt((int)((base + turn_cmd) * LEFT_PWM_SCALE),  0, 255);
      int rightPWM = clampInt((int)((base - turn_cmd) * RIGHT_PWM_SCALE), 0, 255);
      motor.setMotorPWM(leftPWM, rightPWM);

      if (dist >= target_mm) break;
      t_next_ctrl += CTRL_DT_US;
    }

    delayMicroseconds(200);
  }

  motor.setMotorPWM(0,0);
}

/* ============================ Turn Primitives ============================ */

static const float TURN_STOP_RAD   = 0.03f;  // ~1.7°
static const float TURN_SLOW_RAD   = 0.42f;  // taper near target
static const int   PWM_MIN_TURN    = 18;

void turnLeft(int pwm_max = PWM_TURN){
  const float target = +TURN_RAD * ANGLE_SCALE_LEFT;
  odom.reset();
  for(;;){
    odom.update();
    float th = wrap180rad(odom.getTheta());
    float err = wrap180rad(target - th);
    if (fabsf(err) <= TURN_STOP_RAD) break;
    float scale = (fabsf(err) >= TURN_SLOW_RAD) ? 1.f : (fabsf(err)/TURN_SLOW_RAD);
    int pwm = PWM_MIN_TURN + (int)((pwm_max - PWM_MIN_TURN) * scale);
    int dir = (err > 0) ? +1 : -1;
    motor.setMotorPWM((int)(-dir * pwm * LEFT_PWM_SCALE),
                      (int)( dir * pwm * RIGHT_PWM_SCALE));
    delay(3);
  }
  motor.setMotorPWM(0,0);
}

void turnRight(int pwm_max = PWM_TURN){
  const float target = -TURN_RAD * ANGLE_SCALE_RIGHT;
  odom.reset();
  for(;;){
    odom.update();
    float th = wrap180rad(odom.getTheta());
    float err = wrap180rad(target - th);
    if (fabsf(err) <= TURN_STOP_RAD) break;
    float scale = (fabsf(err) >= TURN_SLOW_RAD) ? 1.f : (fabsf(err)/TURN_SLOW_RAD);
    int pwm = PWM_MIN_TURN + (int)((pwm_max - PWM_MIN_TURN) * scale);
    int dir = (err > 0) ? +1 : -1;
    motor.setMotorPWM((int)(-dir * pwm * LEFT_PWM_SCALE),
                      (int)( dir * pwm * RIGHT_PWM_SCALE));
    delay(3);
  }
  motor.setMotorPWM(0,0);
}

/* ============================= Command Runner ============================= */

static void executeCommands(const char *cmds){
  for (int i=0; cmds[i]!='\0'; ++i){
    char c = tolower(cmds[i]);
    switch(c){
      case 'f': forwardOneCell(); break;
      case 'l': turnLeft();       break;
      case 'r': turnRight();      break;
      default: break;
    }
    delay(160);
  }
}

/* ================================= Setup ================================= */

void setup(){
  odom.begin();
  motor.begin();
  lidar.begin();

  // Prime smoothing
  for (int i=0;i<3;i++){
    LidarPosition pos = (i==0)?LEFT:(i==1)?FRONT:RIGHT;
    int r = lidar.readDistance(pos);
    side[i].mm    = (r>=0 && r<=MAX_VALID_MM) ? (float)r : (float)MAX_VALID_MM;
    side[i].valid = (r>=0 && r<=MAX_VALID_MM);
    side[i].t_last_valid = micros();
  }

  // Example path
  executeCommands("fffflfrflflffrflf");

  motor.setMotorPWM(0,0);
}

void loop(){}
