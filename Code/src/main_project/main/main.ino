/***** main.ino : VL6180X L/F/R wall-centering + OLED, ISR-friendly (repulsive when too close) *****/
#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OLED timing
static const uint32_t OLED_DT_US = 100000; // 10 Hz
static uint32_t t_next_oled = 0;

#include "Lidar.hpp"
#include "MotorController.hpp"

// ------------------ VL6180X semantics ------------------
static const int MAX_VALID_MM      = 200;  // sensor max
static const int INVALID_CODE      = -2;   // from Lidar::readDistance()
// ------------------------------------------------------

// ---------------------- Tunables ----------------------
static const int   BASE_FWD_PWM         = 110;  // cruise
static const int   MAX_PWM              = 230;
static const int   MAX_TURN_PWM         = 40;   // cap steering authority
static const int   TURN_SLEW_PER_CYCLE  = 6;    // limit d(turn)/cycle (PWM counts)
static const float SIDE_ALPHA           = 0.45; // EMA when valid (0..1)
static const uint32_t HOLD_INVALID_US   = 100000; // keep last valid for 100 ms
// Corridor behavior
static const int   DESIRED_SIDE_MM      = 65;  // aim distance to wall (used when not "too close")
static const int   TOO_CLOSE_MM         = 40;  // repulsion threshold
static const float PUSH_K               = 1.05f; // extra gain for push-away when too close
// Front gating (all < 200)
static const int   FRONT_SLOW_MM        = 170;
static const int   FRONT_STOP_MM        = 120;
// Optional: slow down when a side is too close
static const int   SIDE_SLOW_BASE       = 80;   // forward PWM cap while pushing away
// ------------------------------------------------------

// ------------------- Timing (no delay) ----------------
static const uint32_t CTRL_DT_US        = 5000;   // 200 Hz control
static const uint32_t LIDAR_DT_US       = 20000;  // ~50 Hz per sensor (staggered)
uint32_t t_next_ctrl = 0;
uint32_t t_next_lidar = 0;
int lidar_round_robin = 0; // 0=LEFT,1=FRONT,2=RIGHT
// ------------------------------------------------------

// Logical indices to match your wiring
enum { LIDAR_LEFT=0, LIDAR_FRONT=1, LIDAR_RIGHT=2 };

MotorController motors;
Lidar lidar;

// Smoothed readings + timestamps
struct Side {
  float  mm = MAX_VALID_MM;  // smoothed value
  bool   valid = false;      // last raw validity
  uint32_t t_last_valid = 0; // micros of last valid update
} side[3];

// State
float turn_cmd = 0.0f;  // signed PWM delta

// --------------- Helpers ---------------
static inline int clampInt(int v, int lo, int hi){ return v<lo?lo : v>hi?hi : v; }
static inline float clamp(float v, float lo, float hi){ return v<lo?lo : v>hi?hi : v; }

// --------------- VL6180X polling (staggered) ---------------
void update_one_lidar(int which) {
  int raw = lidar.readDistance((LidarPosition)which);

  uint32_t now = micros();
  if (raw >= 0 && raw <= MAX_VALID_MM) {
    // Valid: EMA toward raw
    side[which].mm += SIDE_ALPHA * (raw - side[which].mm);
    side[which].valid = true;
    side[which].t_last_valid = now;
  } else {
    // Invalid: hold last valid briefly, then relax to sensor max
    side[which].valid = false;
    if (now - side[which].t_last_valid > HOLD_INVALID_US) {
      side[which].mm += 0.15f * (MAX_VALID_MM - side[which].mm);
    }
  }
}

// --------------- OLED UI ---------------
void oledBegin() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("VL6180X L/F/R"));
  display.display();
}

static inline void printCell(const char *label, int mm, bool valid, bool too_close){
  display.print(label); display.print(':');
  if (!valid) display.print(F("--"));
  else {
    if (mm < 0) mm = 0;
    if (mm > 999) mm = 999;
    display.print(mm);
    if (too_close) display.print('!');
  }
  display.print(' ');
}

void oledUpdate(float dL, bool vL, float dF, bool vF, float dR, bool vR,
                int TOO_CLOSE_MM, int base_pwm, int turn_pwm)
{
  uint32_t now = micros();
  if ((int32_t)(now - t_next_oled) < 0) return; // non-blocking 10 Hz
  t_next_oled = now + OLED_DT_US;

  int iL = (int)(dL + 0.5f);
  int iF = (int)(dF + 0.5f);
  int iR = (int)(dR + 0.5f);

  bool cL = vL && (iL < TOO_CLOSE_MM);
  bool cF = vF && (iF < TOO_CLOSE_MM);
  bool cR = vR && (iR < TOO_CLOSE_MM);

  display.clearDisplay();

  // Row 0: header + PWM/turn
  display.setCursor(0, 0);
  display.print(F("L   F   R"));
  display.setCursor(70, 0);
  display.print(F("PWM "));
  display.print(base_pwm);
  display.print('/');
  display.print(turn_pwm);

  // Row 1: values
  display.setCursor(0, 12);
  printCell("L", iL, vL, cL);
  printCell("F", iF, vF, cF);
  printCell("R", iR, vR, cR);

  // Row 2: legend
  display.setCursor(0, 24);
  display.print(F("--=no ret  !=<thr"));

  display.display();
}

// --------------- Arduino lifecycle ---------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  motors.begin();
  lidar.begin();

  oledBegin();
  t_next_oled = micros();  // start OLED timer

  // Prime readings
  for (int i=0;i<3;i++) {
    int r = lidar.readDistance((LidarPosition)i);
    side[i].mm = (r>=0 && r<=MAX_VALID_MM) ? r : MAX_VALID_MM;
    side[i].valid = (r>=0 && r<=MAX_VALID_MM);
    side[i].t_last_valid = micros();
  }

  t_next_ctrl = micros();
  t_next_lidar = micros();
}

void loop() {
  uint32_t now = micros();

  // --- Staggered lidar updates (one sensor per pass) ---
  if ((int32_t)(now - t_next_lidar) >= 0) {
    update_one_lidar(lidar_round_robin);
    lidar_round_robin = (lidar_round_robin + 1) % 3;
    t_next_lidar += LIDAR_DT_US;
  }

  // --- Control loop at fixed rate, no delay() ---
  if ((int32_t)(now - t_next_ctrl) >= 0) {
    // Distances (smoothed)
    float dL = side[LIDAR_LEFT].mm;
    float dF = side[LIDAR_FRONT].mm;
    float dR = side[LIDAR_RIGHT].mm;

    // Valid if raw-valid OR recently valid (hold window)
    bool vL = side[LIDAR_LEFT].valid  || (now - side[LIDAR_LEFT].t_last_valid  <= HOLD_INVALID_US);
    bool vF = side[LIDAR_FRONT].valid || (now - side[LIDAR_FRONT].t_last_valid <= HOLD_INVALID_US);
    bool vR = side[LIDAR_RIGHT].valid || (now - side[LIDAR_RIGHT].t_last_valid <= HOLD_INVALID_US);

    // --- Base forward speed from front clearance ---
    int base = BASE_FWD_PWM;
    if (dF < FRONT_SLOW_MM && vF) {
      if (dF <= FRONT_STOP_MM) base = 0;
      else {
        float frac = (dF - FRONT_STOP_MM) / float(FRONT_SLOW_MM - FRONT_STOP_MM);
        base = int(BASE_FWD_PWM * clamp(frac, 0.0f, 1.0f));
      }
    }

    // ------------- REPULSIVE LOGIC WHEN TOO CLOSE -------------
    // If a wall is closer than TOO_CLOSE_MM on a side, override and steer away.
    bool closeL = vL && (dL <= TOO_CLOSE_MM);
    bool closeR = vR && (dR <= TOO_CLOSE_MM);

    float err = 0.0f; // mm-equivalent driving the steering

    if (closeL ^ closeR) {
      // Exactly one side is too close -> push away from that side
      if (closeL) {
        // left too close -> steer right (negative err -> leftPWM > rightPWM)
        err = -PUSH_K * (TOO_CLOSE_MM - dL);
      } else {
        // right too close -> steer left (positive err -> rightPWM > leftPWM)
        err = +PUSH_K * (TOO_CLOSE_MM - dR);
      }
      // Optional: slow down while pushing away
      base = min(base, SIDE_SLOW_BASE);

    } else if (closeL && closeR) {
      // Both sides too close: bias toward the side with more clearance
      // (positive err => turn left if right is tighter, negative => turn right if left is tighter)
      float diff = (dL - dR); // if dR < dL -> diff>0 -> steer left, away from right
      err = PUSH_K * diff;
      base = min(base, SIDE_SLOW_BASE);

    } else {
      // ------------- NORMAL FOLLOW/CENTERING when not "too close" -------------
      if (vL && vR) {
        // Centering: steer based on L-R difference (positive => away from right wall)
        err = (dL - dR);
      } else if (vL) {
        // Left-wall follow around setpoint
        err = (DESIRED_SIDE_MM - dL);
      } else if (vR) {
        // Right-wall follow around setpoint
        err = (dR - DESIRED_SIDE_MM);
      } else {
        err = 0.0f; // no info, go straight
      }
    }

    // Proportional steering (units: PWM counts)
    const float KP = 0.35f; // tune as needed
    float turn_target = clamp(KP * err, -float(MAX_TURN_PWM), +float(MAX_TURN_PWM));

    // Slew-limit turn to avoid "keeps turning" tails
    float dturn = turn_target - turn_cmd;
    if (dturn >  TURN_SLEW_PER_CYCLE) dturn =  TURN_SLEW_PER_CYCLE;
    if (dturn < -TURN_SLEW_PER_CYCLE) dturn = -TURN_SLEW_PER_CYCLE;
    turn_cmd += dturn;

    // Compose wheel PWMs (diff drive)
    int leftPWM  = clampInt(base - int(turn_cmd), 0, MAX_PWM);
    int rightPWM = clampInt(base + int(turn_cmd), 0, MAX_PWM);

    // Hard stop if really close in front
    if (vF && dF <= FRONT_STOP_MM) { leftPWM = 0; rightPWM = 0; turn_cmd = 0; }

    motors.setMotorPWM(leftPWM, rightPWM);

    // OLED update now that base/turn are known
    oledUpdate(dL, vL, dF, vF, dR, vR, TOO_CLOSE_MM, base, (int)turn_cmd);

    // Light-weight debug (10 Hz)
    static uint32_t t_dbg = 0;
    if (now - t_dbg > 100000) {
      t_dbg = now;
      Serial.print("L/F/R: ");
      Serial.print((int)dL); Serial.print('/');
      Serial.print((int)dF); Serial.print('/');
      Serial.print((int)dR);
      Serial.print("  close L/R: ");
      Serial.print(closeL); Serial.print('/'); Serial.print(closeR);
      Serial.print("  base: "); Serial.print(base);
      Serial.print("  err: "); Serial.print(err, 1);
      Serial.print("  turn: "); Serial.println(turn_cmd, 1);
    }

    t_next_ctrl += CTRL_DT_US; // fixed cadence
  }

  // Nothing blocking here; ISR for encoders stays responsive.
}
