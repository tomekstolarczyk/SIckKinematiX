#include <stdio.h>
#include "kinematics.h"

int main()
{
    Matrix44 identity, test, result;

    // test czy dziala tworzenie jednostkowych
    create4x4IdentityMatrix(&identity);
    printf("Identity Matrix:\n");
    printMatrix44(&identity);

    // test czy dziala tworzenie macierzy transformacji MDH
    createMDHTransformMatrix(0.5,0.5,0.5,0.5,&test);
    printf("MDH Matrix:\n");
    printMatrix44(&test);

    // test czy dziala mnozenie macierzy
    multiply4x4Matrix(&identity, &test, &result);
    printf("Multiplication result:\n");
    printMatrix44(&result);

    return 0;
}