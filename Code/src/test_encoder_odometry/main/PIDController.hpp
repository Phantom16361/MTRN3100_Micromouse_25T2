/**************************************************************
 *  File         : PIDController.hpp
 *  Author       : Jason E Tomczyk
 *  Description  : Header for velocity PID controller module.
 *                 Includes factory methods for left/right wheels,
 *                 and interface for error-based or measurement-
 *                 based derivative logic.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Added configurable PID with velocity deadband,
 *             smoothing filter, and static instantiation.
 *************************************************************/

#pragma once
#include "robot_param.hpp"

/**
 * @brief General-purpose PID controller with embedded configuration.
 *
 * Supports:
 * - Derivative filtering (EMA)
 * - Deadband threshold
 * - Output clamping
 * - Derivative on measurement or error
 * - Derivative freeze on zero setpoint (useful for hold mode)
 *
 * Instantiation via:
 *   PIDController leftPID = PIDController::Left();
 *   PIDController rightPID = PIDController::Right();
 */
class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    /// Factory constructor for left wheel PID controller
    static PIDController Left();

    /// Factory constructor for right wheel PID controller
    static PIDController Right();

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float minVal, float maxVal);
    void setDerivativeSmoothing(float smoothingAlpha);
    void reset();
    void enableDerivativeFreezeOnZeroSP(bool enable);
    void setVelocityDeadband(float threshold);
    void setUseDerivativeOnMeasurement(bool enable);
    void setTargetSetpoint(float sp);

    /**
     * @brief Runs the PID compute step.
     * @param error = setpoint - measurement
     * @param measurement = actual measured value (used for derivative-on-measurement)
     * @return control effort (clamped)
     */
    float compute(float error, float measurement);

private:
    float Kp, Ki, Kd;
    float integral = 0;
    float previousError = 0;
    float lastMeasurement = 0;
    float filteredDerivative = 0;

    float outputMin = -255;
    float outputMax = 255;
    float alpha = 0.1f;
    float deadband = 0.0f;

    bool useDerivativeOnMeasurement = false;
    bool freezeDWhenSPZero = false;
    float lastTargetSetpoint = 0;
};
