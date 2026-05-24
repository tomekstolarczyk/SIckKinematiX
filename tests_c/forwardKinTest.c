#include <stdio.h>
#include "../src/forwardKinematics.h"

// Prawdziwe parametry Modified DH (Craig) dla robota UR5
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