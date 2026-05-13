#include "pureMath.h"

#ifndef forwardKinematics_h
#define forwardKinematics_h

typedef struct {
    double a[6];
    double alpha[6];
    double d[6];
    double offsets[6];
} RobotArm6DoF;


void createMDHTransformMatrix(double a, double alpha, double d, double theta, Matrix44* result);
void forwardKinematics(const RobotArm6DoF* robotArm, const double* thetas, Matrix44* results);

#endif
