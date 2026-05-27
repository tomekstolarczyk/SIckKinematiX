#include "buildRobotArm.h""

#ifndef forwardKinematics_h
#define forwardKinematics_h

void forwardKinematics(const RobotArm6DoF* robotArm, const double* thetas, Matrix44* results);

#endif
