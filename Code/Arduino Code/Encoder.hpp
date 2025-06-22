#pragma once

#include <Arduino.h>

namespace mtrn3100 {

// The encoder class is a simple interface which counts and stores an encoder's count.
// Encoder pin 1 is attached to the interrupt on the Arduino and used to trigger the count.
// Encoder pin 2 is attached to any digital pin and used to derive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates.
// COUNTS HOW MUCH WHEELS BE SPINNING
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2)
      : encoder1_pin(enc1),
        encoder2_pin(enc2),
        counts_per_revolution(0),
        count(0)
    {
        instance = this;  // Store the instance pointer
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);
        // Attach interrupt on channel A (enc1) rising edge
        attachInterrupt(
          digitalPinToInterrupt(encoder1_pin),
          readEncoderISR,
          RISING
        );
    }

    // Encoder function used to update the encoder
    void readEncoder() {
        noInterrupts();
        // Increase or decrease the count based on the reading on encoder pin 2 (channel B)
        if (digitalRead(encoder2_pin)) {
            ++count;
        } else {
            --count;
        }
        interrupts();
    }

    // Helper function to convert encoder count to revolutions
    float getRotation() {
        // Convert encoder count to rotations: count / counts_per_revolution
        return (counts_per_revolution > 0)
               ? (float)count / (float)counts_per_revolution
               : 0.0f;
    }

private:
    static void readEncoderISR() {
        if (instance != nullptr) {
            instance->readEncoder();
        }
    }

public:
    const uint8_t  encoder1_pin;
    const uint8_t  encoder2_pin;
    volatile long  count;
    uint16_t       counts_per_revolution; // Set this in your setup to your encoder CPR

private:
    static Encoder* instance;
};

// Initialize static instance pointer
Encoder* Encoder::instance = nullptr;

}  // namespace mtrn3100