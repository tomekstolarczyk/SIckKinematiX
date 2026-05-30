#include <stdio.h>
#include "justMath.h"

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

void multiply4x4Matrix(const Matrix44* M1, const Matrix44* M2, Matrix44* result)
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
    
    for(size_t r = 0; r < 4; r++) // wiersz
    {
        for(size_t el = 0; el < 4; el++) // element
        {
            double leftMatrixEl = M1->data[4*r+el];
            for(size_t c = 0; c < 4; c++) // kolumna
            {
                (temp.data[r*4+c]) += ((leftMatrixEl)*(M2->data[c+el*4]));
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

// ----------------------------------------------------------
// 6x6 MATRIX MATH--------------------------------------------
// ----------------------------------------------------------

void printMatrix66(const Matrix66* M) 
{
    printf("----------------------------------------------------------------------------------\n");
    for (size_t r = 0; r < 6; r++)
    {
        printf("| %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f |\n",
            M->data[r*6+0], 
            M->data[r*6+1], 
            M->data[r*6+2],
            M->data[r*6+3], 
            M->data[r*6+4], 
            M->data[r*6+5]);
    }
    printf("----------------------------------------------------------------------------------\n");
}

void multiply6x6Matrix(const Matrix66* M1, const Matrix66* M2, Matrix66* result)
{
    Matrix66 temp = {0};

    for(size_t r=0;r<6;r++) // rows
    {
        for(size_t i = 0; i<6;i++) // elements
        {
            double leftMatrixValue = M1->data[r*6+i]; // constant in deepest part of the loop

            for(size_t c = 0; c<6; c++) // columns
            {
                temp.data[r*6+c] += leftMatrixValue*M2->data[i*6+c];
            }
        }
    }
    *result = temp;
}

void transpose6x6Matrix(const Matrix66* M, Matrix66* result)
{
    Matrix66 temp = {0};

    for(size_t i =0; i<6; i++)
    {
        for(size_t j = 0 ; j<6 ; j++)
        {
            temp.data[i*6+j] = M->data[j*6+i];
        }
    }

    *result = temp;
}

int gaussJordan66MatrixInversion(const Matrix66* M, Matrix66* result)
{
    // finding inverse of a matrix using gauss-jordan method with partial pivoting for numerical precision

    // 1. build the augmented matrix
    double AI[72] = {0};
    int indexJedynki = 0;

    for(size_t r = 0; r<6;r++) // rows
    {
        for (size_t c = 0; c<6;c++) // A 
        {       
            AI[r*12+c] = M->data[r*6+c];
        }
        
        AI[r*12+6+indexJedynki] = 1;
        indexJedynki += 1;
    }

    // 2. the actualy algorithm - turn A into I
    for(size_t c = 0; c<6; c++) // for each column
    {
        // 3. partial pivoting
        double pivot = fabs(AI[c*12+c]); // default choice
        size_t pivotOldRow = c;

        for(size_t r = c+1; r < 6; r++) // start with r=c+1 or else we'll destroy previous steps
        {
            if(pivot < fabs(AI[r*12+c])) 
            {
                pivot = fabs(AI[r*12+c]);
                pivotOldRow = r;
            }
        }

        // 4. place pivot in the right place if needed - exchange rows
        if(pivotOldRow != c)
        {
            for(size_t i = 0; i<12; i++)
            {
                double temp = AI[c*12+i];
                AI[c*12+i] = AI[pivotOldRow*12+i];
                AI[pivotOldRow*12+i] = temp;
            }
        }

        // pivot standing on (c,c) now as desired

        // watch out for singularities
        if(pivot < 1e-7) {return 0;}

        // 5. scale pivot to 1 - divide the entire row by it
        if(pivot != 1.0) 
        {
            double div = AI[c*12+c];
            for(size_t i = 0; i<12; i++)
            {
                AI[c*12+i] = (double)AI[c*12+i]/div; // nie przez pivot bo pivot jest |abs|
            }
        }

        // 6. elimation at last - zero everything except from the pivot
        for(size_t r = 0; r < 6; r++)
        {
            if(r==c) {continue;} // skip pivot row
            double factor = AI[r*12+c];
            for(size_t i = 0; i <12; i++) // substraction operation on the entire row
            {  
                // nowy wiersz = stary wiersz - (factor * wiersz pivota)
                AI[r*12+i] -= factor*AI[c*12+i];
            }
        }
    }

    // find A^(-1)
    for(size_t r = 0; r<6; r++)
    {
        for (size_t c = 0; c<6; c++)
        {
            result->data[6*r+c] = AI[12*r+(6+c)];
        }
    }

    return 1;
}

int calculateDLSInverseCoeff(const Matrix66* J, double lambda, Matrix66* J_DLS_Invert)
{
    // ostateczny wzor (J^T * J + lambda^2 * I)^-1 * J^T
    Matrix66 J_T, Mult_JT_J, Mult_JT_J_inv, result;

    // 1 : J^T * J 
    transpose6x6Matrix(J, &J_T);
    multiply6x6Matrix(&J_T, J, &Mult_JT_J);

    // 2 : J^T * J + lambda^2 * I
    double lambda_sq = lambda*lambda;
    for(size_t i = 0; i < 6; i++)
    {
        Mult_JT_J.data[6*i + i] += lambda_sq;
    }

    // 3: (J^T * J + lambda^2 * I)^-1
    if(gaussJordan66MatrixInversion(&Mult_JT_J, &Mult_JT_J_inv) == 0) {return 0;}

    // 4 ostateczny wzor : (J^T * J + lambda^2 * I)^-1 * J^T
    multiply6x6Matrix(&Mult_JT_J_inv, &J_T, J_DLS_Invert);

    return 1;
}