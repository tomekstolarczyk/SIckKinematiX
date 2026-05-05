#include <stdio.h>
#include "kinematics.h"

void createIdentityMatrix(Matrix44* M)
{
    for(size_t i = 0; i<16; i++)
    {
        if (i == 0 || i == 5 || i == 10 || i == 15) {(*M).data[i]=1.0;}
        else {(*M).data[i]=0.0;}
    }
}