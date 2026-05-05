#ifndef kinematics_h 
#define kinematics_h

typedef struct {
    double data[16];
} Matrix44;

void createIdentityMatrix(Matrix44* M);

void multiply4x4Matrix(Matrix44* M1, Matrix44* M2, Matrix44* result);

void createMDHParametersMatrix(double a, double alpha, double d, double theta, Matrix44* result);

void ForwardKinematics(const double thetas[6], Matrix44* result);

#endif

