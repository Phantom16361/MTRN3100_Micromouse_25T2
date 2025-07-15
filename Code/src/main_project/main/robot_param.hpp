#ifndef ROBOT_PARAM_HPP
#define ROBOT_PARAM_HPP

// ========== Physical Constants (mm, rad, etc.) ==========

// Diameter of the wheel (adjust as needed)
#define WHEEL_DIAMETER_MM  32.4
#define WHEEL_RADIUS_MM    (WHEEL_DIAMETER_MM / 2.0)

// Distance between wheel centers (axle length)
#define AXLE_LENGTH_MM     90

// Encoder resolution
#define TICKS_PER_REV      700

// ========== Derived Constants ==========
#define WHEEL_CIRCUM_MM    (WHEEL_DIAMETER_MM * 3.14159)
#define MM_PER_TICK        (WHEEL_CIRCUM_MM / TICKS_PER_REV)

// ========== Control Limits ==========
#define MAX_PWM_OUTPUT     255
#define MIN_PWM_OUTPUT    -255

// ========== Motion Profiles (optional) ==========
#define MAX_LINEAR_SPEED_MM_S     300.0
#define MAX_ANGULAR_SPEED_RAD_S   4.0

#endif // ROBOT_PARAM_HPP
