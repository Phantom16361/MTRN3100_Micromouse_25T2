#include "EncoderOdometry.hpp"

volatile long EncoderOdometry::leftTicks = 0;
volatile long EncoderOdometry::rightTicks = 0;

EncoderOdometry::EncoderOdometry(float wheelRadiusMM, float axleLengthMM, int ticksPerRevolution)
    : radius(wheelRadiusMM),
      axleLength(axleLengthMM),
      ticksPerRev(ticksPerRevolution),
      x(0.0), y(0.0), theta(0.0),
      prevLeftTicks(0), prevRightTicks(0),
      lastUpdateTime(0)
{
    mmPerTick = (2.0 * 3.14159 * radius) / ticksPerRev;
}

void EncoderOdometry::begin() {
    pinMode(ENC1_A, INPUT);
    pinMode(ENC1_B, INPUT);
    pinMode(ENC2_A, INPUT);
    pinMode(ENC2_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), handleLeftA, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), handleRightA, RISING);

    lastUpdateTime = millis();
}

void EncoderOdometry::reset() {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();

    x = 0;
    y = 0;
    theta = 0;
    prevLeftTicks = 0;
    prevRightTicks = 0;
    lastUpdateTime = millis();
}

void EncoderOdometry::update() {
    long left, right;
    noInterrupts();
    left = leftTicks;
    right = rightTicks;
    interrupts();

    long dLeft = left - prevLeftTicks;
    long dRight = right - prevRightTicks;

    prevLeftTicks = left;
    prevRightTicks = right;

    float dL_mm = dLeft * mmPerTick;
    float dR_mm = dRight * mmPerTick;

    float dCenter = (dL_mm + dR_mm) / 2.0;
    float dTheta = (dR_mm - dL_mm) / axleLength;

    theta += dTheta;
    x += dCenter * cos(theta);
    y += dCenter * sin(theta);

    lastUpdateTime = millis();
}

float EncoderOdometry::getX() const { return x; }
float EncoderOdometry::getY() const { return y; }
float EncoderOdometry::getTheta() const { return theta; }

long EncoderOdometry::getLeftTicks() const { return leftTicks; }
long EncoderOdometry::getRightTicks() const { return rightTicks; }

float EncoderOdometry::getLeftSpeedMMs() const {
    static unsigned long lastTime = 0;
    static long lastTicks = 0;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    if (dt <= 0.0) return 0.0;

    long currentTicks = leftTicks;
    float speed = (currentTicks - lastTicks) * mmPerTick / dt;

    lastTicks = currentTicks;
    lastTime = now;

    return speed;
}

float EncoderOdometry::getRightSpeedMMs() const {
    static unsigned long lastTime = 0;
    static long lastTicks = 0;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    if (dt <= 0.0) return 0.0;

    long currentTicks = rightTicks;
    float speed = (currentTicks - lastTicks) * mmPerTick / dt;

    lastTicks = currentTicks;
    lastTime = now;

    return speed;
}


void EncoderOdometry::handleLeftA() {
    if (digitalRead(ENC1_B))
        leftTicks--;
    else
        leftTicks++;
}

void EncoderOdometry::handleRightA() {
    if (digitalRead(ENC2_B))
        rightTicks++;
    else
        rightTicks--;
}
