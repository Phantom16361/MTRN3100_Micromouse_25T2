#pragma once

#include <math.h>
// ROBOT BRAIN THINGO
namespace mtrn3100 {

class BangBangController {
public:
    BangBangController(float speed, float deadband) : speed(speed), deadband(deadband) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
        error = setpoint - (input - zero_ref);

        if (fabs(error) <= deadband) {
        output = 0.0f;  // close enough, stop
        } else if (error > 0) {
            output = speed;  // go forward
        } else {
             if (fabs(error) < 0.05f) {  // about 35 counts
            output = 0.0f;
        } else {
            output = -speed;
        }
        }

        // TODO: IMPLIMENT BANG BANG CONTROLLER - REFER TO THE TUTORIAL SLIDES
        
        return output;
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    float getError() {
      return error;
    }

    // Setting function used to update internal parameters
    void tune(float speed, float deadband) {
      speed = speed;
      deadband = deadband;
    }

    // This must be called before trying to achieve a setpoint.
    // First argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        zero_ref = zero;
        setpoint = target;
    }

private:
    float speed, deadband;
    float error, output;
    float setpoint = 0;
    float zero_ref = 0;
};

}  // namespace mtrn3100