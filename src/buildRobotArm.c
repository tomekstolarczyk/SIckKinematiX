#include <stdlib.h>
#include "buildRobotArm.h"

void createMDHTransformMatrix(double a, double alpha, double d, double theta, Matrix44* result)
{
    double ct = cos(theta);
    double ca = cos(alpha);
    double st = sin(theta);
    double sa = sin(alpha);

    /*
    ksiazkowy wzorek - "Intro to robotics (J. Craig)"
    [ ct,        -st,         0,   a        ]
    [ st * ca,    ct * ca,   -sa, -d * sa   ]
    [ st * sa,    ct * sa,    ca,  d * ca   ]
    [ 0,          0,          0,   1        ]
    */

    // wiersz 1 
    result -> data[0] = ct;
    result -> data[1] = -st;
    result -> data[2] = 0;
    result -> data[3] = a;

    // wiersz 
    result -> data[4] = st*ca;
    result -> data[5] = ct*ca;
    result -> data[6] = -sa;
    result -> data[7] = -d*sa;

    // wiersz 3 
    result -> data[8] = st*sa;
    result -> data[9] = ct*sa;
    result -> data[10] = ca;
    result -> data[11] = d*ca;

    // wiersz 4
    result -> data[12] = 0;
    result -> data[13] = 0;
    result -> data[14] = 0;
    result -> data[15] = 1;
}