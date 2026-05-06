#include <stdio.h>
#include <math.h>

#include "kinematics.h"

void create4x4IdentityMatrix(Matrix44* M)
{
    for(size_t i = 0; i<16; i++)
    {
        if (i == 0 || i == 5 || i == 10 || i == 15) {(*M).data[i]=1.0;}
        else {(*M).data[i]=0.0;}
    }
}

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

void printMatrix44(const Matrix44* M) 
{
    printf("--------------------------------------------\n");

    for (size_t r = 0; r < 4; r++)
    {
        printf("| %10.4f %10.4f %10.4f %10.4f |\n",
            M->data[r*4+0],
            M->data[r*4+1],
            M->data[r*4+2],
            M->data[r*4+3]);
    }

    printf("--------------------------------------------\n");

}

void multiply4x4Matrix(Matrix44* M1, Matrix44* M2, Matrix44* result)
{
    /*
    Wzorek poglądowy 

    [ a1 b1 c1 d1 ]   [ a2 b2 c2 d2 ]   [ a1a2 + e1e2 + i1i2 + m1m2, ..., ..., ... ]
    [ e1 f1 g1 h1 ] * [ e2 f2 g2 h2 ] = [                                          ]
    [ i1 j1 k1 l1 ]   [ i2 j2 k2 l2 ]   [                                          ]
    [ m1 n1 o1 p1 ]   [ m2 n2 o2 p2 ]   [                                          ] 

    */

    // tworzymy przejsciowa macierz do pracy zeby nie miec problemow z 
    // nadpisywaniem samego siebie przy mnozeniu macierzy w ciagu
    // A = A * B 
    // multiply4x4Matrix(result, &joint_matrix, result);
    Matrix44 temp = {0};
    // empty - initalized ^ 
    
    for(size_t r = 0; r < 4; r++) // dla kazdego wiersza macierzy A
    {
        for(size_t c = 0; c < 4; c++) // wymnazamy go przez kolumny macierzy B
        {
            for(size_t el = 0; el < 4; el++) //iloczyn skalarny wiersz x kolumna
            {
                (temp.data[r*4+c]) += ((M1->data[4*r+el])*(M2->data[c+el*4]));
            }
        }
    }

    *result = temp;
}

