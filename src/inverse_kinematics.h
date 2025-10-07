#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "configs.h"

class InverseKinematics {
public:
    InverseKinematics();
    void calculateWheelSpeeds(float vx, float vy, float vt, float* result);

private:
    const float WHEEL_ANGLES[4];
    float jacobian[4][3];
};

#endif
