#include <stdio.h>

#include "../src/forwardKinematics.h"

RobotArm6DoF ramie = {
    {0, 0, -0.4, -0.2, 0, 0}, 
    {0, M_PI/2.0, 0, 0, -M_PI/2.0, M_PI/2.0},
    {0.6, 0, 0, 0.1, 0, 0},
    {M_PI/2.0, -M_PI/2.0, 0, M_PI/2.0, 0, M_PI/2.0}
};

int main(void)
{
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