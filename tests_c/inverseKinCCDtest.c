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

    // Cel
    double thetas_target[] = {0.5, 0.3, -0.4, 0.8, 1.2, -0.2};
    Matrix44 fkResults[6];
    forwardKinematics(&ramie, thetas_target, fkResults);
    Vector3D target = { fkResults[5].data[3], fkResults[5].data[7], fkResults[5].data[11] };

    // Test
    double thetas_test[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // ZAKTUALIZOWANE WYWOLANIE (dodano max_iters i tolerance)
    inverseKinematicsCCD(&ramie, thetas_test, &target, 5000, 0.001);

    forwardKinematics(&ramie, thetas_test, fkResults);
    double ex = target.x - fkResults[5].data[3];
    double ey = target.y - fkResults[5].data[7];
    double ez = target.z - fkResults[5].data[11];
    double blad = sqrt(ex*ex + ey*ey + ez*ez);

    assert(blad < 0.005);
    printf("[PASS] inverseKinCCDtest\n");
    return 0;
}