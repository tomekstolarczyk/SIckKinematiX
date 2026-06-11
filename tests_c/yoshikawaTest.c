#include <stdio.h>
#include <assert.h>
#include "../src/jacobian.h"

int main(void) {
    RobotArm6DoF ramie = {
        {0.0, 0.0, -0.425, -0.39225, 0.0, 0.0}, 
        {0.0, M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0},
        {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    Matrix66 J;
    double thetas_good[] = {0.1, -1.0, 1.5, 0.5, 0.5, 0.5};
    calculateJacobian(&ramie, thetas_good, &J);
    double w_good = calculateManipulabilityYoshikawaIndex(&J);

    double thetas_bad[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    calculateJacobian(&ramie, thetas_bad, &J);
    double w_bad = calculateManipulabilityYoshikawaIndex(&J);

    assert(w_good > w_bad);

    printf("[PASS] yoshikawaTest\n");
    return 0;
}