#pragma once

#include <Arduino.h>

#include <Adafruit_GFX.h>

#include <Adafruit_SSD1306.h>   // <-- NEW (needs the Adafruit_SSD1306 library)
 
namespace mtrn3100 {
 
class EncoderOdometry {

public:

    /**

     * @param radius        Wheel radius [m]

     * @param wheelBase     Wheel-to-wheel distance [m]

     * @param invertLeft    true → multiply left-wheel ticks by −1 once

     * @param invertRight   true → multiply right-wheel ticks by −1 once

     */

    EncoderOdometry(float radius,

                    float wheelBase,

                    bool invertLeft  = false,

                    bool invertRight = false)

        : x(0), y(0), h(0),

          R(radius), B(wheelBase),

          signL(invertLeft  ? -1 : 1),

          signR(invertRight ? -1 : 1),

          lastL(0), lastR(0),

          oled(nullptr) {}                        // <-- NEW (OLED pointer)
 
    /* Attach an already-initialised Adafruit_SSD1306.                *

     * Example in your .ino:  odo.attachDisplay(&display);            */

    void attachDisplay(Adafruit_SSD1306* disp) { oled = disp; }
 
    /* Zero the pose (optional helper). */

    void reset(float newX = 0, float newY = 0, float newH = 0) {

        noInterrupts();

        x = newX;  y = newY;  h = wrapPi(newH);

        lastL = lastR = 0;

        interrupts();

    }
 
    /* Call once per loop with *cumulative* wheel angles [rad]. */

    void update(float leftValue, float rightValue)

    {

        // 1. wheel-angle increments

        float dPhiL =  signL * (leftValue  - lastL);

        float dPhiR =  signR * (rightValue - lastR);
 
        // 2. wheel-arc lengths

        float dSL = R * dPhiL;

        float dSR = R * dPhiR;
 
        // 3. body-frame increments

        float dS     = 0.5f * (dSL + dSR);

        float dTheta = (dSR - dSL) / B;
 
        // 4. midpoint integration

        float midT = h + 0.5f * dTheta;

        x += dS * cosf(midT);

        y += dS * sinf(midT);

        h  = wrapPi(h + dTheta);
 
        // 5. remember totals

        lastL = leftValue;

        lastR = rightValue;
 
        // 6. OPTIONAL: show on OLED if one is attached

        if (oled) {
    oled->clearDisplay();
    oled->setTextSize(1);
    oled->setTextColor(SSD1306_WHITE);
 
    oled->setCursor(0, 0);
    oled->print(F("x "));      oled->print(x, 2);          // two decimals
 
    oled->setCursor(0, 10);
    oled->print(F("y "));      oled->print(y, 2);
 
    oled->setCursor(0, 20);
    oled->print(F("h "));      oled->print(h, 2);
 
    oled->display();           // push buffer to OLED
}

    }
 
    // ───── getters ────────────────────────────────────────────────

    float getX() const { return x; }

    float getY() const { return y; }

    float getH() const { return h; }
 
private:

    // pose

    volatile float x, y, h;
 
    // geometry

    const float R, B;

    const int8_t signL, signR;
 
    // last absolute wheel angles

    volatile float lastL, lastR;
 
    // OLED handle (null until user attaches one)

    Adafruit_SSD1306* oled;
 
    static float wrapPi(float a) {

        while (a >  PI) a -= TWO_PI;

        while (a <= -PI) a += TWO_PI;

        return a;

    }

};
 
} // namespace mtrn3100

 