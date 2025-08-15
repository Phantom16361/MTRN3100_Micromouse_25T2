// main.ino

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "pin_config.hpp"
#include "robot_param.hpp"
#include "MotorController.hpp"
#include "IMUOdometry.hpp"
#include "PIDController.hpp"

// OLED configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// PID gains
const float KP = 1.5f, KI = 0.1f, KD = 0.3f;
PIDController yawPID(KP, KI, KD);

// Motion thresholds
const int   MAX_PWM       =  200;
const int   MIN_PWM_DEAD  =   15;    // ↑ increased to overcome stiction
const float DRIFT_DEG     =   0.0f;
const float INIT_TOL      =   0.5f;
const int   INIT_PWM_SPD  =   30;

// Phase flags & yaw refs
static bool firstMove      = true;
static bool didRecalibrate = false;
float initialYaw, setpointYaw;
unsigned long lastTime = 0;

MotorController motor;
IMUOdometry      imu;

// wrap angle into (–180,180]
static float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize OLED
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize IMU (performs bias calibration)
  imu.begin(1, 0);
  delay(100);
  imu.update();

  // Initialize motors
  motor.begin();

  // Record start yaw and compute setpoint (–90°)
  initialYaw  = imu.getYawDegrees();
  setpointYaw = wrap180(initialYaw - 90.0f);

  // Configure PID
  yawPID.setGains(KP, KI, KD);
  yawPID.setOutputLimits(-MAX_PWM, MAX_PWM);
  yawPID.setDerivativeSmoothing(0.1f);
  yawPID.setUseDerivativeOnMeasurement(true);
  yawPID.reset(initialYaw);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) * 0.001f;
  lastTime = now;

  imu.update();
  float rawYaw = imu.getYawDegrees();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Yaw: ");
  display.print(rawYaw, 1);
  display.display();

  if (firstMove) {
    float err = wrap180(setpointYaw - rawYaw);
    if (fabs(err) > INIT_TOL) {
      int dir = (err > 0) ? 1 : -1;
      motor.setMotorPWM(-dir * INIT_PWM_SPD, dir * INIT_PWM_SPD);
    } else {
      motor.setMotorPWM(0, 0);
      firstMove = false;
      yawPID.reset(rawYaw);          // <-- use new overload to seed lastMeasurement
    }
    return;
  }

  // PHASE 2: fine PID correction
  float error = wrap180(setpointYaw - rawYaw);
  float u     = yawPID.compute(error, rawYaw, dt);  // <-- pass dt in the right slot now
  int pwm     = (int)u;

  if (!didRecalibrate && fabs(error) < DRIFT_DEG) {
    imu.begin(1, 0);
    delay(100);
    imu.update();
    rawYaw = imu.getYawDegrees();
    yawPID.reset(rawYaw);            // <-- same here
    didRecalibrate = true;
    motor.setMotorPWM(0, 0);
    return;
  }

  if      (pwm > 0 && pwm < MIN_PWM_DEAD)  pwm =  MIN_PWM_DEAD;
  else if (pwm < 0 && pwm > -MIN_PWM_DEAD) pwm = -MIN_PWM_DEAD;
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  motor.setMotorPWM(-pwm, +pwm);
}