#ifndef vectorMath_h
#define vectorMath_h

// odejmowanie wektorowe 
void myVectorSub(const double* a, const double* b, double* out);

// iloczyny skalarny, wektorowy i dlugosc wektorowa
double myVectorDotProduct(const double* a, const double* b);
void myVectorCrossProduct(const double* a, const double* b, double* out);
double myVectorLength(const double* v);

// normalizacja i rzutowanie na plaszczyzne
void myVectorNormalization(double* v);
void myProjectPlane(const double* v, const double* normal, double* out);

#endif