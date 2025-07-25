/**************************************************************
 *  File         : CubicTrajectory.cpp
 *  Author       : Jason E Tomczyk
 *  Description  : Generates cubic time-based trajectories for
 *                 smooth motion planning from an initial state
 *                 to a target state over a fixed duration.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Implemented cubic coefficient solver and accessors
 *             for position, velocity, and acceleration profiles.
 *************************************************************/

#include "CubicTrajectory.hpp"

void CubicTrajectory::generate(float x0, float v0, float xf, float vf, float T) {
    a0 = x0;
    a1 = v0;
    a2 = (3 * (xf - x0) / (T * T)) - (2 * v0 + vf) / T;
    a3 = (2 * (x0 - xf) / (T * T * T)) + (v0 + vf) / (T * T);
}

float CubicTrajectory::getPosition(float t) const {
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t;
}

float CubicTrajectory::getVelocity(float t) const {
    return a1 + 2 * a2 * t + 3 * a3 * t * t;
}

float CubicTrajectory::getAcceleration(float t) const {
    return 2 * a2 + 6 * a3 * t;
}
