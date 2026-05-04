#ifndef kinematics_h 
#define kinematics_h

typedef struct {
    double data[16];
} Matri4x4;

void createIdentityMatrix(Matri4x4* M);
void multiply4x4Matrix(Matri4x4* M1, Matri4x4* M2, Matri4x4* result);
void createMDHParametersMatrix(double a, double alpha, double d, double theta, Matri4x4* result);
void ForwardKinematics(const double thetas[6], Matri4x4* result);

#endif