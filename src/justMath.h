#include <math.h>

#ifndef pureMath_h
#define pureMath_h

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// 4x4 MATRIX MATH (needed for forward kinematics using the MDH matricies) -------------------------------------------

typedef struct {
    double data[16];
} Matrix44;

void create4x4IdentityMatrix(Matrix44* M);
void printMatrix44(const Matrix44* M); 
void multiply4x4Matrix(const Matrix44* M1, const Matrix44* M2, Matrix44* result);

// 3d VECTOR MATH (needed for the CCD algorithm) --------------------------------------------

typedef struct{
    double x;
    double y;
    double z;
} Vector3D;

// odejmowanie wektorowe 
void myVectorSub(const Vector3D* v1, const Vector3D* v2, Vector3D* vout);

// iloczyny skalarny, wektorowy i dlugosc wektorowa
double myVectorDotProduct(const Vector3D* v1, const Vector3D* v2);
double myVectorLength(const Vector3D* v);
void myVectorCrossProduct(const Vector3D* v1, const Vector3D* v2, Vector3D* vout);

// normalizacja i rzutowanie na plaszyczne
void myVectorNormalization(Vector3D* v);
void myVectorPlaneProjection(const Vector3D* v, const Vector3D* n, Vector3D* vout);

// 6x6 MATRIX MATH (for Jacobian math - workspace analysis and Inverse Kinematics DLL) -------------------------------------------

typedef struct {
    double data[36];
} Matrix66;

void printMatrix66(const Matrix66* M); 
void multiply6x6Matrix(const Matrix66* M1, const Matrix66* M2, Matrix66* result);
void transpose6x6Matrix(const Matrix66* M, Matrix66* result);
// int not void cause with this we want to know if the operation finished with success
// as not every matrix is invertable
int gaussJordan66MatrixInversion(const Matrix66* M, Matrix66* result);
int calculateDLSInverse(const Matrix66* J, double lambda, Matrix66* J_DLS_Invert);

#endif

