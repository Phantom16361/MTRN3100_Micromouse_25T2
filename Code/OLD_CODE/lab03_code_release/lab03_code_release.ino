#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define CPR 700
#define WHEEL_DIAMETER_MM 32.4
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.1416)

#define MOT1PWM  9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

volatile long countL = 0;
volatile long countR = 0;

// ========== ENCODER INTERRUPTS ==========
void readEncoderL() {
  if (digitalRead(4)) ++countL;
  else{
     --countL;
  } 
}

void readEncoderR() {
  if (digitalRead(5)) ++countR;
  else --countR;
}

// ========== MOTOR CONTROL ==========
void setMotor(int pwm_pin, int dir_pin, int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) digitalWrite(dir_pin, HIGH);
  else {
    digitalWrite(dir_pin, LOW);
    pwm = -pwm;
  }
  analogWrite(pwm_pin, pwm);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // OLED INIT
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("System Init...");
  display.display();
  delay(1000);

  // Encoder Pins
  pinMode(2, INPUT_PULLUP);  // Left encoder A
  pinMode(4, INPUT_PULLUP);  // Left encoder B
  pinMode(3, INPUT_PULLUP);  // Right encoder A
  pinMode(5, INPUT_PULLUP);  // Right encoder B

  attachInterrupt(digitalPinToInterrupt(2), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(3), readEncoderR, RISING);

  // Motor Pins
  pinMode(MOT1PWM, OUTPUT);
  pinMode(MOT1DIR, OUTPUT);
  pinMode(MOT2PWM, OUTPUT);
  pinMode(MOT2DIR, OUTPUT);
}

void loop() {

  float brake_buffer = 0.9;  // Rev buffer before the actual target
  float target_revs = 1.964; // Target = ~200 mm
  float deadband = 0.02;
  // Compute revolutions
  float revL = (float)countL / CPR;
  float revR = (float)countR / CPR;

  // Compute distance
  float distL = revL * WHEEL_CIRCUMFERENCE_MM;
  float distR = revR * WHEEL_CIRCUMFERENCE_MM;

  // --- MOTOR CONTROL LOGIC (Bang-Bang style) ---
  target_revs = 1.96; // for ~200 mm

  int16_t pwmL = 0;
  int16_t pwmR = 0;
  deadband = 0.03;

  if (revL < target_revs - brake_buffer) pwmL = 120;  // Cruise speed
    else if (revL < target_revs - deadband) pwmL = 80;  // Slow approach
    else {
      pwmL = 0;  // Stop
      pwmR = 0;
    } 

  if (revR < target_revs - brake_buffer) pwmR = 120;
    else if (revR < target_revs - deadband) pwmR = 80;
    else{
      pwmR = 0;
      pwmL = 0;
    } 



  setMotor(MOT1PWM, MOT1DIR, pwmL);
  setMotor(MOT2PWM, MOT2DIR, pwmR);

  // --- SERIAL MONITOR OUTPUT ---
  Serial.print("L: Count ");
  Serial.print(countL);
  Serial.print(" | Rev ");
  Serial.print(revL, 3);
  Serial.print(" | mm ");
  Serial.print(distL, 1);

  Serial.print("    ||    ");

  Serial.print("R: Count ");
  Serial.print(countR);
  Serial.print(" | Rev ");
  Serial.print(revR, 3);
  Serial.print(" | mm ");
  Serial.println(distR, 1);

  // --- OLED DISPLAY OUTPUT ---
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("L: ");
  display.print(countL);
  display.print(" | ");
  display.println(distL, 0);

  display.print("R: ");
  display.print(countR);
  display.print(" | ");
  display.println(distR, 0);

  if (pwmL == 0 && pwmR == 0) {
    display.println("Target Reached!");
  }

  display.display();

  
}