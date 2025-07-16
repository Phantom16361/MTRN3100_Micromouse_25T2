#pragma once

/**
 * @brief Simple position PID controller for 1D linear motion.
 *
 * Computes control output (e.g., motor PWM) based on the error between a
 * target position (set via setTarget) and the current position.
 * Integrates over time and includes derivative damping.
 *
 * Usage example:
 * ```cpp
 * PositionController posPID(1.0, 0.01, 0.1);
 * posPID.setTarget(200);   // target position in mm
 *
 * loop {
 *   float pwm = posPID.update(currentX_mm, dt);
 *   motor.setPWM(pwm);
 *   if (posPID.isFinished()) stop();
 * }
 * ```
 */
class PositionController {
public:
    /**
     * @brief Constructor to initialize PID gains.
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    PositionController(float kp, float ki, float kd);

    /**
     * @brief Sets a new target position (in mm).
     * This resets internal integrator and derivative state.
     * @param targetMM Desired final position in millimeters
     */
    void setTarget(float targetMM);

    /**
     * @brief Computes the control output based on current position.
     * @param currentPosMM Current position in mm
     * @param dt Time since last update (in seconds)
     * @return Control output (e.g., PWM command), unbounded
     */
    float update(float currentPosMM, float dt);

    /**
     * @brief Returns true when the last position error is within ±1.5 mm.
     * Use to stop the robot when close enough to the goal.
     */
    bool isFinished() const;

    /**
     * @brief Resets internal state (integral, error, etc.) without changing the target.
     */
    void reset();

private:
    float kp, ki, kd;

    float target;
    float integral;
    float lastError;
    bool  firstRun;
};
