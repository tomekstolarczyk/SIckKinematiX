#include <stdio.h>

#include "forwardKinematics.h"
#include "vector3DMath.h"

void inverseKinematicsCCD(double* thetas, const Vector3D* target)
{
    Matrix44 fkResults[6];
    int max_iters = 100;
    double tolerance = 0.001;

    // 1. iteracje zewnetrzne 
    for(size_t iter = 0; iter < max_iters ; iter++)
    {
        // sprawdzamy gdzie obecnie sie znajdujemy
        forwardKinematics(fkResults, thetas);
        Vector3D currentEndEffectorPosition = {fkResults[5].data[3], fkResults[5].data[7], fkResults[5].data[11]};

        // sprawdzamy czy cel trafiony
        Vector3D distanceVector;
        myVectorSub(&currentEndEffectorPosition, target, &distanceVector);
        if(myVectorLength(&distanceVector)<=tolerance)
        {
            printf("made it in %d", iter);
            break;
        }

        // 2. jesli cel nietrafiony to petla wewnetrzna po stawach od konca do bazy
        for(size_t i = 5 ; i>=0; i--)
        {
            // TODO
        }
    }
}