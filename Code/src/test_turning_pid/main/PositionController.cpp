/**************************************************************
 *  File         : PositionController.cpp
 *  Author       : Jason E Tomczyk
 *  Description  : Implementation of a simple position PID controller
 *                 using a fixed target and tunable gains. Intended for
 *                 forward driving or wall-distance setpoint tracking.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Initial control structure and error-tolerant stop logic.
 *************************************************************/

#include "PositionController.hpp"
#include <Arduino.h>

PositionController::PositionController(float kp_, float ki_, float kd_)
  : kp(kp_), ki(ki_), kd(kd_), target(0), integral(0), lastError(0), firstRun(true) {}

/**
 * Sets a new desired target position in mm.
 * Clears internal integrator and derivative memory.
 */
void PositionController::setTarget(float targetMM) {
    target = targetMM;
    integral = 0;
    lastError = 0;
    firstRun = true;
}

/**
 * Computes the control output using the PID algorithm.
 * @param currentPosMM Current position of the robot (in mm)
 * @param dt Time since last update (in seconds)
 * @return Output control effort (typically passed to motor PWM)
 */
float PositionController::update(float currentPosMM, float dt) {
    float error = target - currentPosMM;
    integral += min(error,30) * dt;

    float derivative = 0;
    if (!firstRun) {
        derivative = (error - lastError) / dt;
    } else {
        firstRun = false;
    }

    lastError = error;

    float output = kp * error + ki * integral + kd * derivative;
    return output;
}

/**
 * Returns true if the last error is within a 1.5 mm tolerance band.
 */
bool PositionController::isFinished() const {
    return abs(lastError) < 1.5;
}

/**
 * Clears accumulated integral and derivative state.
 * Use this if the robot is reset or the controller is reused.
 */
void PositionController::reset() {
    integral = 0;
    lastError = 0;
    firstRun = true;
}
