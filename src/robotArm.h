#include "pureMath.h"

#ifndef robotArm_h
#define robotArm_h

typedef struct {
    double a[6];
    double alpha[6];
    double d[6];
    double offsets[6];
} RobotArm6DoF;

void createMDHTransformMatrix(double a, double alpha, double d, double theta, Matrix44* result);

#endif
