#include "../src/forwardKinematics.h"

#ifndef ramieRobota_h
#define ramieRobota_h

    // hardcoded wymiary ramion robota
    static const double l1 = 0.6;
    static const double l2 = 0.5;
    static const double l3 = 0.4;
    static const double l4 = 0.3;
    static const double l5 = 0.2;
    static const double l6 = 0.1;

    static RobotArm6DoF ramie = {
        {0, 0, -l3, -l5, 0, 0},
        {0, M_PI/2.0, 0, 0, -M_PI/2.0, M_PI/2.0},
        {l1, 0, 0, l6, 0, 0},
        {M_PI/2.0, -M_PI/2.0, 0, M_PI/2.0, 0, M_PI/2.0}
    };

#endif