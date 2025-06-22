#include "Encoder.hpp"
#include "Motor.hpp"
#include "BangBangController.hpp"

#define MOT1PWM  9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

mtrn3100::Motor        motor(MOT1PWM, MOT1DIR);
mtrn3100::Motor        motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder      encoder(2, 4);
mtrn3100::BangBangController controller(100.0f, 0.02f);
//             ^^^^^   ^^^^^^^
//            speed   deadband

unsigned long lastTime = 0;
float lastRev = 0.0;
float peakRPM = 0.0;



void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting closed-loop control...");

  Serial.println("Waiting...");
  delay(500);
  encoder.counts_per_revolution = 700;       // 700 counts = 1 rev
  controller.zeroAndSetTarget(
    encoder.getRotation(),                   // zero at current pos
    1.96f                                     // target = +2 rev
  );
}

void loop() {
  float rev    = encoder.getRotation();      // in revolutions
  unsigned long now = millis();  // time in ms

  float dt = (now - lastTime) / 1000.0;  // convert to seconds
  float dRev = rev - lastRev;

  float rpm = 0;
  if (dt > 0.1) {  // only update if at least 100ms has passed
    rpm = (dRev / dt) * 60.0;  // rev/sec → rev/min
    lastTime = now;
    lastRev = rev;
  }

  if (rpm > peakRPM) {
    peakRPM = rpm;
  }
  float output = controller.compute(rev);    // ±255 or 0

  // Drive both motors
  motor .setPWM((int16_t)output);
  motor2.setPWM((int16_t)output);

  // Monitor
  Serial.print("Count: ");
  Serial.print(encoder.count);
  Serial.print("   Rev: ");
  Serial.print(rev, 3);
  Serial.print("   PWM: ");
  Serial.println(output, 0);
  Serial.print("   RPM: ");
  Serial.println(rpm, 1);

  long count = encoder.count;
float wheel_diameter_mm = 33.4;  // your wheel diameter here
float wheel_circumference_mm = wheel_diameter_mm * 3.14159;

float distance_mm = (float)count / encoder.counts_per_revolution * wheel_circumference_mm;

Serial.print("Count: ");
Serial.print(count);
Serial.print("   Distance (mm): ");
Serial.println(distance_mm, 1);

  // Once within deadband, output==0 ⇒ motors stop
  if (fabs(controller.getError()) <= 0.01f) {
   
    motor.setPWM(0);
    motor2.setPWM(0);
    while (true) { /* halt here */ }
  }

  delay(50);
}