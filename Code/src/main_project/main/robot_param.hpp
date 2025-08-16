/**************************************************************
 *  File         : robot_param.hpp
 *  Author       : Jason E Tomczyk
 *  Description  : Global constants for robot dimensions and
 *                 tuning parameters (e.g., PID gains).
 *                 Used across control, odometry, and motion planning.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Declared physical constants (wheel size, axle length),
 *             tick conversion formulas, and separate PID parameters
 *             for left and right velocity loops.
 *************************************************************/

#ifndef ROBOT_PARAM_HPP
#define ROBOT_PARAM_HPP

// ========== Physical Constants (mm, rad, etc.) ==========

// Diameter of the wheel (adjust as needed)
#define WHEEL_DIAMETER_MM  32.4f
#define WHEEL_RADIUS_MM    (WHEEL_DIAMETER_MM / 2.0)

// Distance between wheel centers (axle length)
#define AXLE_LENGTH_MM     82

// Encoder resolution
#define TICKS_PER_REV      700

// ========== Derived Constants ==========
#define WHEEL_CIRCUM_MM    (WHEEL_DIAMETER_MM * 3.14159)
#define MM_PER_TICK        (WHEEL_CIRCUM_MM / TICKS_PER_REV)

// ========== Control Limits ==========
#define MAX_PWM_OUTPUT     255
#define MIN_PWM_OUTPUT    -255

// ========== Motion Profiles (optional) ==========
#define MAX_LINEAR_SPEED_MM_S     300.0f
#define MAX_ANGULAR_SPEED_RAD_S   4.0f

/* 
*  Only edit PID values when absolutely necessary
*  Ensure all values have been tuned using testing profiles
*  before further commiting. 
*/
// ========== PID Control Limits ==========
#define PID_DERIV_SMOOTH   0.10f   // Exponential smoothing alpha
#define PID_DEADBAND       0.10f   // mm/s tolerance

#define PID_OUTPUT_MIN    -255.0
#define PID_OUTPUT_MAX     255.0

#define CONTROL_INTERVAL_MS 25
#define CONTROL_DT          (CONTROL_INTERVAL_MS / 1000.0f)

// ========== Velocity PID Values ==========
#define LEFT_VEL_KP  1.85f
#define LEFT_VEL_KI  0.28f
#define LEFT_VEL_KD  0.025f

#define RIGHT_VEL_KP 1.85f
#define RIGHT_VEL_KI 0.28f
#define RIGHT_VEL_KD 0.02f

// ========== Position PID Values ==========
#define LEFT_POS_KP 1.10f
#define LEFT_POS_KI 0.30f
#define LEFT_POS_KD 0.10f

#define RIGHT_POS_KP 1.10f
#define RIGHT_POS_KI 0.30f
#define RIGHT_POS_KD 0.10f



#endif // ROBOT_PARAM_HPP
