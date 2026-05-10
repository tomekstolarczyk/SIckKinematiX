#ifndef vectorMath_h
#define vectorMath_h

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
void myVectorProjectOnPlane(const Vector3D* v, const Vector3D* n, Vector3D* vout);



#endif