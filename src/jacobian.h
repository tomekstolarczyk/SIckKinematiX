#include "forwardKinematics.h"

#ifndef jacobian_h
#define jacobian_h

void calculateJacobian(const RobotArm6DoF* ramie, const double* thetas, Matrix66* jacobianResultMatrix);

#endif