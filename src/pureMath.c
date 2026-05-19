#include <stdio.h>
#include <math.h>

#include "pureMath.h"

// ----------------------------------------------------------
// 4x4 MATRIX MATH ------------------------------------------
// ----------------------------------------------------------

void create4x4IdentityMatrix(Matrix44* M)
{
    for(size_t i = 0; i<16; i++)
    {
        if (i == 0 || i == 5 || i == 10 || i == 15) {(*M).data[i]=1.0;}
        else {(*M).data[i]=0.0;}
    }
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

// ----------------------------------------------------------
// 3d VECTOR MATH--------------------------------------------
// ----------------------------------------------------------

// odejmowanie wektorowe 
void myVectorSub(const Vector3D* v1, const Vector3D* v2, Vector3D* vout)
{
    vout->x = v1->x - v2->x;
    vout->y = v1->y - v2->y;
    vout->z = v1->z - v2->z;
}

// iloczyny skalarny
double myVectorDotProduct(const Vector3D* v1, const Vector3D* v2)
{
    return (v1->x * v2->x) + (v1->y * v2->y) + (v1->z * v2->z);
}

// dlugosc wektorowa
double myVectorLength(const Vector3D* v)
{
    return sqrt(myVectorDotProduct(v,v));
}

// iloczyn wektorowy
void myVectorCrossProduct(const Vector3D* v1, const Vector3D* v2, Vector3D* vout)
{   
    // podejscie analityczne
    vout->x = (v1->y)*(v2->z) - (v1->z)*(v2->y);
    vout->y = (v1->z)*(v2->x) - (v1->x)*(v2->z);
    vout->z =  (v1->x)*(v2->y) - (v1->y)*(v2->x);
}

// normalizacja 
void myVectorNormalization(Vector3D* v)
{
    double length = myVectorLength(v);

    // epsilon maszynowy - zabezpieczamy przed dzieleniem przez zero
    if(length>1e-7)
    {
        v->x = (double)v->x/length;
        v->y = (double)v->y/length;
        v->z = (double)v->z/length;
    }   
}

void myVectorPlaneProjection(const Vector3D* v, const Vector3D* n, Vector3D* vout)
{
    // wedlug wzoru na rzutowanie ortogonalne
    // (oryginalny wektor v minus część wzdłuż osi normalnej)

    double dot = myVectorDotProduct(v,n);

    // odejmujemy wektorowo
    vout->x = v->x - (dot * n->x);
    vout->y = v->y - (dot * n->y);
    vout->z = v->z - (dot * n->z);
}
