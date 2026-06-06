#include "forwardKinematics.h"
#include "jacobian.h"

#ifndef inverseKinematics
#define inverseKinematics

void inverseKinematicsCCD(RobotArm6DoF* ramie, double* thetas, const Vector3D* target, int max_iters, double tolerance);
void inverseKinematicsDLS(RobotArm6DoF* ramie, double* thetas, const Matrix44* targetPose, int max_iters, double tolerance, double step_size, double lambda);

#endif