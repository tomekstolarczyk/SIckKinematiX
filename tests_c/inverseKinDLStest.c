#include <stdio.h>
#include <math.h>
#include "../src/inverseKinematics.h"
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
    Matrix44 fkResults[6];

    // ################################################ PRZYPADEK TESTOWY 1 ################################################ 
    printf("\n--- CASE TEST 1: Standardowy cel (osiagalny) ---\n");

    double thetas_target_1[] = {0.5, 0.3, -0.4, 0.8, 1.2, -0.2};
    forwardKinematics(&ramie, thetas_target_1, fkResults);
    
    Matrix44 targetPose_1 = fkResults[5]; 

    printf("CEL DO OSIAGNIECIA (X, Y, Z): %9.4f, %9.4f, %9.4f\n", 
           targetPose_1.data[3], targetPose_1.data[7], targetPose_1.data[11]);
    
    printf("ODPALAM DLS (Startuje z thetas = 0)...\n");
    double thetas_test_1[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    inverseKinematicsDLS(&ramie, thetas_test_1, &targetPose_1);

    forwardKinematics(&ramie, thetas_test_1, fkResults);
    Matrix44 achievedPose_1 = fkResults[5];
    
    printf("POZYCJA KONCOWA (X, Y, Z): %9.4f, %9.4f, %9.4f\n", 
           achievedPose_1.data[3], achievedPose_1.data[7], achievedPose_1.data[11]);

    // Blad pozycji (odl euklidesowa)
    double ex_1 = targetPose_1.data[3] - achievedPose_1.data[3];
    double ey_1 = targetPose_1.data[7] - achievedPose_1.data[7];
    double ez_1 = targetPose_1.data[11] - achievedPose_1.data[11];
    double bladXYZ_1 = sqrt(ex_1*ex_1 + ey_1*ey_1 + ez_1*ez_1);

    // Bład rotacji (iloczyny skalarne - jeśli osie się nakładają, suma wyniesie 3.0)
    double dotX_1 = targetPose_1.data[0]*achievedPose_1.data[0] + targetPose_1.data[4]*achievedPose_1.data[4] + targetPose_1.data[8]*achievedPose_1.data[8];
    double dotY_1 = targetPose_1.data[1]*achievedPose_1.data[1] + targetPose_1.data[5]*achievedPose_1.data[5] + targetPose_1.data[9]*achievedPose_1.data[9];
    double dotZ_1 = targetPose_1.data[2]*achievedPose_1.data[2] + targetPose_1.data[6]*achievedPose_1.data[6] + targetPose_1.data[10]*achievedPose_1.data[10];
    double bladRot_1 = 3.0 - (dotX_1 + dotY_1 + dotZ_1);

    printf("BLAD POZYCJI: %.6f\n", bladXYZ_1);
    printf("BLAD ROTACJI (odchylenie od 3.0): %.6f\n", bladRot_1);

    if(bladXYZ_1 < 0.001 && bladRot_1 < 0.001) { printf("CASE 1: LETSGOOO\n"); }
    else { printf("CASE 1: not today :(\n"); }


    // ################################################ PRZYPADEK TESTOWY 2 ################################################ 
    printf("\n--- CASE TEST 2: Pulapka 180 stopni (martwy punkt LWP) ---\n");

    double thetas_start_2[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    forwardKinematics(&ramie, thetas_start_2, fkResults);
    Matrix44 targetPose_2 = fkResults[5]; 

    // Oś X zostaje tak samo, oś Y i Z patrzą w całkowicie przeciwną stronę.
    targetPose_2.data[1] = -targetPose_2.data[1]; // Y.x
    targetPose_2.data[5] = -targetPose_2.data[5]; // Y.y
    targetPose_2.data[9] = -targetPose_2.data[9]; // Y.z

    targetPose_2.data[2] = -targetPose_2.data[2]; // Z.x
    targetPose_2.data[6] = -targetPose_2.data[6]; // Z.y
    targetPose_2.data[10]= -targetPose_2.data[10]; // Z.z
    
    // lekkie przesuniecie zeby cos bylo
    targetPose_2.data[11] -= 0.1; 

    printf("ODPALAM DLS Z PULAPKA...\n");
    
    double thetas_test_2[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    inverseKinematicsDLS(&ramie, thetas_test_2, &targetPose_2);

    forwardKinematics(&ramie, thetas_test_2, fkResults);
    Matrix44 achievedPose_2 = fkResults[5];

    double ex_2 = targetPose_2.data[3] - achievedPose_2.data[3];
    double ey_2 = targetPose_2.data[7] - achievedPose_2.data[7];
    double ez_2 = targetPose_2.data[11] - achievedPose_2.data[11];
    double bladXYZ_2 = sqrt(ex_2*ex_2 + ey_2*ey_2 + ez_2*ez_2);

    double dotX_2 = targetPose_2.data[0]*achievedPose_2.data[0] + targetPose_2.data[4]*achievedPose_2.data[4] + targetPose_2.data[8]*achievedPose_2.data[8];
    double dotY_2 = targetPose_2.data[1]*achievedPose_2.data[1] + targetPose_2.data[5]*achievedPose_2.data[5] + targetPose_2.data[9]*achievedPose_2.data[9];
    double dotZ_2 = targetPose_2.data[2]*achievedPose_2.data[2] + targetPose_2.data[6]*achievedPose_2.data[6] + targetPose_2.data[10]*achievedPose_2.data[10];
    double bladRot_2 = 3.0 - (dotX_2 + dotY_2 + dotZ_2);

    printf("BLAD POZYCJI: %.6f\n", bladXYZ_2);
    printf("BLAD ROTACJI (odchylenie od 3.0): %.6f\n", bladRot_2);

    if(bladXYZ_2 < 0.005 && bladRot_2 < 0.005) { printf("CASE 2: LETSGOOO!\n"); }
    else { printf("CASE 2: not today :( tak mialo byc - widac ze LWP do spatchowania\n"); }

    return 0;
}