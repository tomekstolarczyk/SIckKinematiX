#include <stdio.h>
#include "jacobian.h"

void calculateJacobian(const RobotArm6DoF* ramie, const double* thetas, Matrix66* jacobianResultMatrix)
{
    
    // idziemy w petli przez kazdy staw i liczymy geometrycznie Jakobian
    // zgodnie ze wzorem : https://robotics.stackexchange.com/questions/16759/jacobian-of-a-6dof-arm

    Matrix44 fkResults[6];
    forwardKinematics(ramie, thetas, fkResults);

    Vector3D endEfektor = {fkResults[5].data[3], fkResults[5].data[7], fkResults[5].data[11]};

    for(size_t i = 0; i < 6; i++)
    {
        Vector3D rotationAxis = {fkResults[i].data[2], fkResults[i].data[6], fkResults[i].data[10]};
        Vector3D currPosition = {fkResults[i].data[3], fkResults[i].data[7], fkResults[i].data[11]};

        Vector3D R;
        myVectorSub(&endEfektor, &currPosition, &R);
        
        Vector3D jacobianVelocity;
        myVectorCrossProduct(&rotationAxis, &R, &jacobianVelocity);

        // wypelniamy kolumny macierzy wynikowej
        jacobianResultMatrix->data[i] = jacobianVelocity.x;
        jacobianResultMatrix->data[6+i] = jacobianVelocity.y;
        jacobianResultMatrix->data[2*6+i] = jacobianVelocity.z;
        jacobianResultMatrix->data[3*6+i] = rotationAxis.x;
        jacobianResultMatrix->data[4*6+i] = rotationAxis.y;
        jacobianResultMatrix->data[5*6+i] = rotationAxis.z;
    }
}

double calculateManipulabilityYoshikawaIndex(const Matrix66* J)
{
    // Yoshikawa Manipulability Index :
    // >    w = sqrt(detJ*detJ^T)
    // but for square Jacobians (as for our robot) simplifies to 
    // >    w = |detJ| 

    return fabs(calculate6x6MatrixDeterminant(J));
}