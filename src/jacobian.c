#include <stdio.h>

#include "jacobian.h"


void calculateJacobian(const RobotArm6DoF* ramie, const double* thetas, Matrix66* jacobianResultMatrix)
{
    
    // idziemy w petli przez kazdy staw i liczymy geometrycznie Jakobian
    // zgodnie ze wzorem : https://robotics.stackexchange.com/questions/16759/jacobian-of-a-6dof-arm

    Matrix44* fkResults[6];
    forwardKinematics(ramie, thetas, fkResults);
    for(size_t i = 0; i < 6; i++)
    {

    }
}