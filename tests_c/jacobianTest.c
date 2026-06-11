#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../src/jacobian.h"

int main(void) {
    RobotArm6DoF ramie = {
        {0.0, 0.0, -0.425, -0.39225, 0.0, 0.0}, 
        {0.0, M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0},
        {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    double thetas[] = {0.5, 0.3, -0.4, 0.8, 1.2, -0.2};
    Matrix66 jacobian, inverted, identityCheck;
    
    calculateJacobian(&ramie, thetas, &jacobian);
    int status = gaussJordan66MatrixInversion(&jacobian, &inverted);
    
    assert(status == 1); // Odwracanie musi sie udac dla dobrej pozy

    multiply6x6Matrix(&jacobian, &inverted, &identityCheck);
    for(int i = 0; i < 6; i++) {
        assert(fabs(identityCheck.data[i * 6 + i] - 1.0) < 1e-6); 
    }

    printf("[PASS] jacobianTest\n");
    return 0;
}