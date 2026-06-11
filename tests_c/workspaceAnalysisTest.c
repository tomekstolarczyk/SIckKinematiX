#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "../src/workspaceAnalysis.h"

int main(void) {
    RobotArm6DoF ramie = {
        {0.0, 0.0, -0.425, -0.39225, 0.0, 0.0}, 
        {0.0, M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0},
        {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    size_t points = 1000;

    double* x = malloc(sizeof(double) * points);
    double* y = malloc(sizeof(double) * points);
    double* z = malloc(sizeof(double) * points);
    double* yoshikawas = malloc(sizeof(double) * points);
    
    assert(x != NULL && y != NULL && z != NULL && yoshikawas != NULL);

    workspaceAnalysis(&ramie, points, x, y, z, yoshikawas);

    // Upewniamy sie, ze cos zostalo zapisane (wartosci nie sa NaN i indeks istnieje)
    assert(yoshikawas[0] >= 0.0);
    
    free(x); free(y); free(z); free(yoshikawas);

    printf("[PASS] workspaceAnalysisTest\n");
    return 0;
}