#include "forwardKinematics.h"

#ifndef jacobian_h
#define jacobian_h

typedef struct {
    double data[36];
} Matrix66;

void calculateJacobian(const RobotArm6DoF* ramie, const double* thetas, double* jacobianResultMatrix);

#endif