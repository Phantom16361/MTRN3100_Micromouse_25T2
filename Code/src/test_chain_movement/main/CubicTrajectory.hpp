/**************************************************************
 *  File         : CubicTrajectory.hpp
 *  Author       : Jason E Tomczyk
 *  Description  : Header for cubic polynomial trajectory generator.
 *                 Provides interface to generate and evaluate
 *                 smooth motion paths over time.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Declared generate(), getPosition(), getVelocity(),
 *             and getAcceleration() for time-based motion profiles.
 *************************************************************/

#pragma once

class CubicTrajectory {
public:
    void generate(float x0, float v0, float xf, float vf, float T);
    float getPosition(float t) const;
    float getVelocity(float t) const;
    float getAcceleration(float t) const;

private:
    float a0, a1, a2, a3;
};
