#include <Arduino.h>
#include "pin_config.hpp"
#include "robot_param.hpp"
#include "MotorController.hpp"
#include "EncoderOdometry.hpp"

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH  OLED_WIDTH
#define SCREEN_HEIGHT OLED_HEIGHT

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MotorController motor;
EncoderOdometry odom(WHEEL_RADIUS_MM, AXLE_LENGTH_MM, TICKS_PER_REV);

int cycleCount = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Motor + Odometry OLED Test Starting...");

  motor.begin();
  odom.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED + Motor Ready");
  display.display();
}

void printToOLED(const char* label, int leftPWM, int rightPWM) {
  display.clearDisplay();
  display.setCursor(0, 0);

  // Top: status and PWM
  display.print(label); display.print(" | L:");
  display.print(leftPWM); display.print(" R:");
  display.println(rightPWM);

  // Middle: position (X, Y, θ)
  display.print("X: "); display.print(odom.getX(), 0); display.print("mm ");
  display.print("Y: "); display.print(odom.getY(), 0); display.println("mm");
  display.print("Th: "); display.print(odom.getTheta(), 2); display.println("rad");

  // Bottom: ticks
  display.print("L:"); display.print(odom.getLeftTicks());
  display.print(" R:"); display.print(odom.getRightTicks());

  display.display();
}

void loop() {
  odom.update();
  ++cycleCount;

  // Drive forward
  Serial.println("Forward");
  motor.setMotorPWM(150, 150);
  for (int i = 0; i < 10; ++i) {
    odom.update();
    printToOLED("FWD", 150, 150);
    delay(100);
  }

  // Stop
  Serial.println("Stop");
  motor.setMotorPWM(0, 0);
  for (int i = 0; i < 10; ++i) {
    odom.update();
    printToOLED("STOP", 0, 0);
    delay(100);
  }

  // Drive backward
  Serial.println("Backward");
  motor.setMotorPWM(-150, -150);
  for (int i = 0; i < 10; ++i) {
    odom.update();
    printToOLED("REV", -150, -150);
    delay(100);
  }

  // Stop again
  Serial.println("Stop");
  motor.setMotorPWM(0, 0);
  for (int i = 0; i < 10; ++i) {
    odom.update();
    printToOLED("STOP", 0, 0);
    delay(100);
  }
}
