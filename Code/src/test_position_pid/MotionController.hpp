#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "PIDController.hpp"
#include "CubicTrajectory.hpp"
#include "robot_param.hpp"

class MotionController {
public:
    MotionController();

    void begin();
    void moveForward(float distance_mm, float duration_s);
    void update(unsigned long currentMillis);
    bool isFinished() const;
    float getX() const;  // Optional getter for debug

private:
    EncoderOdometry odom;
    MotorController motor;
    PIDController leftPID;
    PIDController rightPID;
    CubicTrajectory trajectory;

    unsigned long startTime = 0;
    float startX = 0.0;
    bool active = false;
};


#endif
