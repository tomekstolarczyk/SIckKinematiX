#include <stdio.h>
#include "../src/jacobian.h"

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
    Matrix66 J;
    
    // ---------------------------------------------------------
    // TEST 1: Dobra, zgieta poza (duza swoboda)
    // ---------------------------------------------------------
    double thetas_good[] = {0.1, -1.0, 1.5, 0.5, 0.5, 0.5};
    calculateJacobian(&ramie, thetas_good, &J);
    
    double w_good = calculateManipulabilityYoshikawaIndex(&J);
    printf("--- TEST 1: ZGIETY ROBOT (BEZPIECZNA POZA) ---\n");
    printf("Wskaznik Yoshikawy: %f\n\n", w_good);

    // ---------------------------------------------------------
    // TEST 2: Osobliwosc (Singularity - wyprostowane ramie)
    // ---------------------------------------------------------
    double thetas_bad[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    calculateJacobian(&ramie, thetas_bad, &J);
    
    double w_bad = calculateManipulabilityYoshikawaIndex(&J);
    printf("--- TEST 2: WYPROSTOWANY ROBOT (OSOBLIWOSC) ---\n");
    printf("Wskaznik Yoshikawy: %f\n\n", w_bad);

    return 0;
}