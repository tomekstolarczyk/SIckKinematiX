#include <stdio.h>
#include "../src/workspaceAnalysis.h"

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
    size_t points = 5000;
    double x[points], y[points], z[points];

    workspaceAnalysis(&ramie, points, x, y, z);

    for(size_t i = 0; i <points; i++)
    {
        printf("point %zu : (%f, %f, %f)\n", i, x[i], y[i], z[i]);
    }
}