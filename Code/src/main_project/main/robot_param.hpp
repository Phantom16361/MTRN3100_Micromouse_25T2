#ifndef ROBOT_PARAM_HPP
#define ROBOT_PARAM_HPP

// ========== Physical Constants (mm, rad, etc.) ==========

// Diameter of the wheel (adjust as needed)
#define WHEEL_DIAMETER_MM  32.4f
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

#define PID_DERIV_SMOOTH   0.10f   // Exponential smoothing alpha
#define PID_DEADBAND       0.10f   // mm/s tolerance

#define PID_OUTPUT_MIN    -255.0
#define PID_OUTPUT_MAX     255.0

#define CONTROL_INTERVAL_MS 25
#define CONTROL_DT          (CONTROL_INTERVAL_MS / 1000.0f)

// ========== Motion Profiles (optional) ==========
#define MAX_LINEAR_SPEED_MM_S     300.0f
#define MAX_ANGULAR_SPEED_RAD_S   4.0f


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
#define LEFT_POS_KD 0.05f

#define RIGHT_POS_KP 1.10f
#define RIGHT_POS_KI 0.30f
#define RIGHT_POS_KD 0.05f



#endif // ROBOT_PARAM_HPP
