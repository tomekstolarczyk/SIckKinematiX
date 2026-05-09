#include <stdio.h>
#include <math.h>

#include "vectorMath.h"

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
