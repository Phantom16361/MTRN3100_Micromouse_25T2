
#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PIDController.hpp"

<<<<<<< Updated upstream
EncoderOdometry odom(WHEEL_RADIUS_MM, AXLE_LENGTH_MM, TICKS_PER_REV);
=======
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
static const int   SIDE_PRESENT_MM      = 130;

// Push-away if really close (cap the push)
static const int   TOO_CLOSE_MM         = 40;
static const int   REPULSE_MAX_PWM      = 14;

// Target lateral balance: use corridor difference (dR - dL)
static const int   CENTER_DEADBAND_MM   = 20;      // big deadband
static const int   CENTER_CONFIRM_CYCLES= 6;       // must persist
static const uint32_t FRESH_WINDOW_US   = 30000;   // both sides fresh

// Very light wall centering gain (mm -> PWM)
static const float SIDE_KP              = 0.00f;   // small on purpose
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
>>>>>>> Stashed changes
MotorController motor;

PIDController leftPID(LEFT_VEL_KP, LEFT_VEL_KI, LEFT_VEL_KD);
PIDController rightPID(RIGHT_VEL_KP, RIGHT_VEL_KI, RIGHT_VEL_KD);

unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL_MS = 25;

float targetSpeedSet = 200.0;
float targetSpeedL = 0.0;
float targetSpeedR = 0.0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Looping Step Input PID Tuner");

  odom.begin();
  motor.begin();

  leftPID.setOutputLimits(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT);
  rightPID.setOutputLimits(MIN_PWM_OUTPUT, MAX_PWM_OUTPUT);

<<<<<<< Updated upstream
  leftPID.setDerivativeSmoothing(0.1);
  rightPID.setDerivativeSmoothing(0.1);
=======
  // Example path
  executeCommands("rrfrflfrflflfrfrflfflfrflfrflfffflfffrfrflflfffflfrflflfrfrfflfflf");
>>>>>>> Stashed changes

  leftPID.setUseDerivativeOnMeasurement(true);
  rightPID.setUseDerivativeOnMeasurement(true);

  leftPID.enableDerivativeFreezeOnZeroSP(true);
  rightPID.enableDerivativeFreezeOnZeroSP(true);

  leftPID.setVelocityDeadband(0.1);
  rightPID.setVelocityDeadband(0.1);
}

void loop() {
  odom.update();
  unsigned long now = millis();

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    // 4s ON, 4s OFF step pattern
    if ((now / 1000) % 8 < 4) {
      targetSpeedL = targetSpeedSet;
      targetSpeedR = targetSpeedSet;
    } else {
      targetSpeedL = 0.0;
      targetSpeedR = 0.0;
    }

    float leftVel = odom.getLeftSpeedMMs();
    float rightVel = odom.getRightSpeedMMs();

    float errorL = targetSpeedL - leftVel;
    float errorR = targetSpeedR - rightVel;

    leftPID.setTargetSetpoint(targetSpeedL);
    rightPID.setTargetSetpoint(targetSpeedR);

    float leftPWM = leftPID.compute(errorL, dt, leftVel);
    float rightPWM = rightPID.compute(errorR, dt, rightVel);

    if (targetSpeedL == 0 && abs(leftVel) < 1.0) {
      leftPID.reset();
      leftPWM = 0;
    }

    if (targetSpeedR == 0 && abs(rightVel) < 1.0) {
      rightPID.reset();
      rightPWM = 0;
    }

    motor.setMotorPWM(leftPWM, rightPWM);

    // Serial plotter output
    Serial.print("L_SP:"); Serial.print(targetSpeedL, 2); Serial.print(" ");
    Serial.print("L_VEL:"); Serial.print(leftVel, 2); Serial.print(" ");
    Serial.print("L_OUT:"); Serial.print(leftPWM, 2); Serial.print(" ");
    Serial.print("R_SP:"); Serial.print(targetSpeedR, 2); Serial.print(" ");
    Serial.print("R_VEL:"); Serial.print(rightVel, 2); Serial.print(" ");
    Serial.print("R_OUT:"); Serial.print(rightPWM, 2); Serial.print(" ");
    Serial.print("REF_Bottom:-100 "); Serial.println("REF_Top:250");
  }
}
