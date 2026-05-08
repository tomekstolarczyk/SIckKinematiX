#include <stdio.h>
#include <math.h>

#include "vectorMath.h"

void myVectorSub(const double* a, const double* b, double* out)
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

double myVectorDotProduct(const double* a, const double* b)
{
    return ((a[0]*b[0])+(a[1]*b[1])+(a[2]*b[2]));
}

void myVectorCrossProduct(const double* a, const double* b, double* out)
{

}

double myVectorLength(const double* v)
{

}

void myVectorNormalization(double* v)
{

}

void myProjectPlane(const double* v, const double* normal, double* out)
{

}

