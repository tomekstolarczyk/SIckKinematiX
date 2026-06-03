#include <stdio.h>
#include <stdlib.h>
#include <time.h> // do mierzenia czasu
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
    size_t points = 1000000;

    double* x = malloc(sizeof(double)*points);
    double* y = malloc(sizeof(double)*points);
    double* z = malloc(sizeof(double)*points);
    if(x == NULL || y == NULL || z == NULL) {return 1;}

    clock_t start = clock();
    workspaceAnalysis(&ramie, points, x, y, z);
    clock_t end = clock();

    double timee = (double) (end - start) / CLOCKS_PER_SEC;

    for(size_t i = 0; i <10; i++)
    {
        printf("point %zu : (%f, %f, %f)\n", i, x[i], y[i], z[i]);
    }

    printf("time spent: %f\n", timee);

    free(x);
    free(y);
    free(z);
    
    return 0;
}