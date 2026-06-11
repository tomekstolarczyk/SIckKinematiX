#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../src/forwardKinematics.h"

int main(void) {
    RobotArm6DoF ramie = {
        {0.0, 0.0, -0.425, -0.39225, 0.0, 0.0}, 
        {0.0, M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0},
        {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    Matrix44 results[6];
    double thetas[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    forwardKinematics(&ramie, thetas, results);

    // Sprawdzamy czy pierwsza macierz ma prawidlowy offset Z i brak niechcianych rotacji
    assert(fabs(results[0].data[11] - 0.089159) < 1e-5); 
    assert(fabs(results[0].data[0] - 1.0) < 1e-5);       

    printf("[PASS] forwardKinTest\n");
    return 0;
}