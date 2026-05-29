#include <stdio.h>
#include "../src/jacobian.h"

int main(void)
{
    RobotArm6DoF ramie = {
        // a - przesunięcia wzdłuż osi X (długości głównych kości)
        {0.0, 0.0, -0.425, -0.39225, 0.0, 0.0}, 
        // alpha - skręcenia wokół osi X (w radianach)
        {0.0, M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0},
        // d - przesunięcia wzdłuż osi Z (wysokość bazy i odsunięcia nadgarstka)
        {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823},
        // offsets - ewentualne początkowe offsety kątowe 
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    double thetas[] = {0.5, 0.3, -0.4, 0.8, 1.2, -0.2};
    Matrix66 jacobian;

    calculateJacobian(&ramie, thetas, &jacobian);

    printMatrix66(&jacobian);

    return 0;

}


