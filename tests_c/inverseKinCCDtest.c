#include <stdio.h>

#include "../src/inverseKinematics.h"

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
    double thetas_target[] = {0.5, 0.3, -0.4, 0.8, 1.2, -0.2};
    Matrix44 fkResults[6];
    forwardKinematics(&ramie, thetas_target, fkResults);

    Vector3D target = {
        fkResults[5].data[3],
        fkResults[5].data[7],
        fkResults[5].data[11]
    };

    printf("CEL DO OSIAGNIECIA (X, Y, Z): %9.4f, %9.4f, %9.4f\n", target.x, target.y, target.z);
    
    printf("ODPALAM CCD\n");
    double thetas_test[] = {0.0,0.0,0.0,0.0,0.0,0.0};
    inverseKinematicsCCD(&ramie, thetas_test, &target);

    // wyniki ccd 
    forwardKinematics(&ramie, thetas_test, fkResults);
    Vector3D achieved = {
        fkResults[5].data[3],
        fkResults[5].data[7],
        fkResults[5].data[11]
    };
    printf("POZYCJA KONCOWA (X, Y, Z): %9.4f, %9.4f, %9.4f\n", achieved.x, achieved.y, achieved.z);

    double ex = target.x - achieved.x;
    double ey = target.y - achieved.y;
    double ez = target.z - achieved.z;
    double blad = sqrt(ex*ex + ey*ey + ez*ez);

    printf("BLAD (odleglosc od celu): %.6f\n", blad);

    if(blad<0.001) {printf("LETSGOOO\n");}
    else { printf("not today :(\n");}

    return 0;

}