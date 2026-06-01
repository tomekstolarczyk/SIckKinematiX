#include "forwardKinematics.h"
#include "jacobian.h"

#ifndef inverseKinematics
#define inverseKinematics

void inverseKinematicsCCD(RobotArm6DoF* ramie, double* thetas, const Vector3D* target); // only xyz position
void inverseKinematicsDLS(RobotArm6DoF* ramie, double* thetas, const Matrix44* targetPose); // both xyz and orentation

#endif