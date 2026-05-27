#include <stdio.h>
#include <math.h>
#include "forwardKinematics.h"

void forwardKinematics(const RobotArm6DoF* robotArm, const double* thetas, Matrix44* results)
{

    //tworzymy macierz jednostkowa zeby od czegos zaczac obliczenia 
    Matrix44 currentM;
    create4x4IdentityMatrix(&currentM);

    // tworzymy macierze transformacji, przemnazamy przez dotychczasowy wynik etc. 
    // wszystko zgodnie z metodyka Manipulator Forward Kinematics ("Intro to robotics", John Craig)
    Matrix44 jointMatrix; 
    double q; // theta + offsets
    for(size_t i = 0; i<6; i++)
    {
        q = (robotArm->offsets[i])+thetas[i];
        createMDHTransformMatrix(robotArm->a[i],robotArm->alpha[i],robotArm->d[i],q,&jointMatrix);
        multiply4x4Matrix(&currentM, &jointMatrix, &currentM);

        // dla inverse kinematics bedziemy chcieli miec wszystkie posrednie wyniki tez
        results[i] = currentM;
    }
}