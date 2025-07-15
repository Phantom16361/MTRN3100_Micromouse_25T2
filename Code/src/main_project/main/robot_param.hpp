#ifndef ROBOT_PARAM_HPP
#define ROBOT_PARAM_HPP

// ========== Physical Constants (mm, rad, etc.) ==========

// Diameter of the wheel (adjust as needed)
#define WHEEL_DIAMETER_MM  32.4
#define WHEEL_RADIUS_MM    (WHEEL_DIAMETER_MM / 2.0)

// Distance between wheel centers (axle length)
#define AXLE_LENGTH_MM     91

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


// ========== Velocity PID Values ==========
#define LEFT_VEL_KP  1.85
#define LEFT_VEL_KI  0.28
#define LEFT_VEL_KD  0.025

#define RIGHT_VEL_KP 1.85
#define RIGHT_VEL_KI 0.28
#define RIGHT_VEL_KD 0.02

// ========== Position PID Values ==========
#define LEFT_POS_KP 1.10
#define LEFT_POS_KI 0.30
#define LEFT_POS_KD 0.05

#define RIGHT_POS_KP 1.10
#define RIGHT_POS_KI 0.30
#define RIGHT_POS_KD 0.05



#endif // ROBOT_PARAM_HPP
