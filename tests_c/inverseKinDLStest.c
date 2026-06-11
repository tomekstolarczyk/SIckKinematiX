#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../src/inverseKinematics.h"
#include "../src/forwardKinematics.h"

int main(void) {
    RobotArm6DoF ramie = {
        {0.0, 0.0, -0.425, -0.39225, 0.0, 0.0}, 
        {0.0, M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0},
        {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    Matrix44 fkResults[6];
    double thetas_target[] = {0.2, 0.1, -0.2, 0.4, 0.5, -0.1};
    forwardKinematics(&ramie, thetas_target, fkResults);
    Matrix44 targetPose = fkResults[5]; 
    
    double thetas_test[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // ZAKTUALIZOWANE WYWOLANIE (max_iters, tolerance, step_size, lambda)
    inverseKinematicsDLS(&ramie, thetas_test, &targetPose, 500, 0.001, 0.1, 0.1);

    forwardKinematics(&ramie, thetas_test, fkResults);
    Matrix44 achievedPose = fkResults[5];

    double ex = targetPose.data[3] - achievedPose.data[3];
    double ey = targetPose.data[7] - achievedPose.data[7];
    double ez = targetPose.data[11] - achievedPose.data[11];
    double bladXYZ = sqrt(ex*ex + ey*ey + ez*ez);
    double dotZ = targetPose.data[2]*achievedPose.data[2] + targetPose.data[6]*achievedPose.data[6] + targetPose.data[10]*achievedPose.data[10];

    assert(bladXYZ < 0.01);
    assert(fabs(dotZ - 1.0) < 0.05);

    printf("[PASS] inverseKinDLStest\n");
    return 0;
}