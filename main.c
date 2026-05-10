#include <stdio.h>
#include "forwardKinematics.h"

int main()
{
    Matrix44 identity, test, result1;
    Matrix44 results[6];

    // test czy dziala tworzenie jednostkowych
    create4x4IdentityMatrix(&identity);
    printf("Identity Matrix:\n");
    printMatrix44(&identity);

    // test czy dziala tworzenie macierzy transformacji MDH
    createMDHTransformMatrix(0.5,0.5,0.5,0.5,&test);
    printf("MDH Matrix:\n");
    printMatrix44(&test);

    // test czy dziala mnozenie macierzy
    multiply4x4Matrix(&identity, &test, &result1);
    printf("Multiplication results:\n");
    printMatrix44(&result1);

    // forward kinematics
    double thetas[] = {0.1,0.2,0.3,0.4,0.5,0.6};
    forwardKinematics(thetas, results);
    printMatrix44(&results[0]);
    printMatrix44(&results[1]);
    printMatrix44(&results[2]);
    printMatrix44(&results[3]);
    printMatrix44(&results[4]);
    printMatrix44(&results[5]);

    return 0;
}