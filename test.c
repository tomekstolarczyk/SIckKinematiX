#include <stdio.h>
#include "forwardKinematics.h"

int main()
{
    // hardcoded wymiary ramion robota
    const double l1 = 0.6;
    const double l2 = 0.5;
    const double l3 = 0.4;
    const double l4 = 0.3;
    const double l5 = 0.2;
    const double l6 = 0.1;

    RobotArm6DoF ramie = {
        {0, 0, -l3, -l5, 0, 0},
        {0, M_PI/2.0, 0, 0, -M_PI/2.0, M_PI/2.0},
        {l1, 0, 0, l6, 0, 0},
        {M_PI/2.0, -M_PI/2.0, 0, M_PI/2.0, 0, M_PI/2.0}
    };
    
    Matrix44 results[6];

    // forward kinematics
    double thetas[] = {0.1,0.2,0.3,0.4,0.5,0.6};
    forwardKinematics(&ramie, thetas, results);
    printf("pierwszy przegub:\n");
    printMatrix44(&results[0]);
    printf("drugi przegub:\n");
    printMatrix44(&results[1]);
    printf("trzeci przegub:\n");
    printMatrix44(&results[2]);
    printf("czwarty przegub:\n");
    printMatrix44(&results[3]);
    printf("piaty przegub:\n");
    printMatrix44(&results[4]);
    printf("szosty przegub:\n");
    printMatrix44(&results[5]);

    return 0;
}