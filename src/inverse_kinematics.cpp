#include "inverse_kinematics.h"
#include <cmath>

InverseKinematics::InverseKinematics() : WHEEL_ANGLES{WHEEL1_ANG, WHEEL2_ANG, WHEEL3_ANG, WHEEL4_ANG} {
    for (int i = 0; i < 4; i++) {
        jacobian[i][0] = sin(WHEEL_ANGLES[i]);
        jacobian[i][1] = cos(WHEEL_ANGLES[i]);
        jacobian[i][2] = ROBOT_RADIUS;
    }
}

void InverseKinematics::calculateWheelSpeeds(float vx, float vy, float vt, float* result) {
    for (int i = 0; i < 4; i++) {
        result[i] = 0.0;
        result[i] += jacobian[i][0] * vx;
        result[i] += jacobian[i][1] * vy;
        result[i] += jacobian[i][2] * vt;
        result[i] = (1.0 / WHEEL_RADIUS) * result[i];
    }
}
