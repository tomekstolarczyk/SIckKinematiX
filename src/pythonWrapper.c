#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "pureMath.h"
#include "forwardKinematics.h"
#include "inverseKinematics.h"

// laczymy kinematyke prosta z c z pythonem

static PyObject* forwardKin(PyObject* self, PyObject* args)
{
    // 1. ROZPAKOWANIE z Pyhona
    double t1, t2, t3, t4, t5, t6;
    if (!PyArg_ParseTuple(args, "dddddd", &t1, &t2, &t3, &t4, &t5, &t6))
        return NULL;

    // 2. liczymy w c
    
    double l1 = 0.6, l2 = 0.5, l3 = 0.4, l4 = 0.3, l5 = 0.2, l6 = 0.1;
    RobotArm6DoF ramie = {
        {0, 0, -l3, -l5, 0, 0},
        {0, M_PI/2.0, 0, 0, -M_PI/2.0, M_PI/2.0},
        {l1, 0, 0, l6, 0, 0},
        {M_PI/2.0, -M_PI/2.0, 0, M_PI/2.0, 0, M_PI/2.0}
    };

    double tetas[] = {t1,t2,t3,t4,t5,t6};
    Matrix44 results[6];

    forwardKinematics(&ramie, tetas, results); 

    // 3. pakujemy calosc spowrotem do pythona

    double x = results[5].data[3];
    double y = results[5].data[7];
    double z = results[5].data[11];

    return Py_BuildValue("ddd", x, y, z);
}

// teraz dla kinematyki odwrotnej ccd

static PyObject* inverseKin(PyObject* self, PyObject* args)
{
    // 1. rozpakowanie 
    double tx, ty, tz;
    if (!PyArg_ParseTuple(args, "ddd", &tx, &ty, &tz))
        return NULL;
    
    // 2. przeliczamy, start z pozycji zerowej
    Vector3D target = {tx, ty, tz};
    double l1 = 0.6, l2 = 0.5, l3 = 0.4, l4 = 0.3, l5 = 0.2, l6 = 0.1;
    RobotArm6DoF ramie = {
        {0, 0, -l3, -l5, 0, 0},
        {0, M_PI/2.0, 0, 0, -M_PI/2.0, M_PI/2.0},
        {l1, 0, 0, l6, 0, 0},
        {M_PI/2.0, -M_PI/2.0, 0, M_PI/2.0, 0, M_PI/2.0}
    };
    double thetas[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    inverseKinematicsCCD(&ramie, thetas, &target);

    // 3. pakujemy
    return Py_BuildValue("dddddd", thetas[0], thetas[1], thetas[2]
    , thetas[3], thetas[4], thetas[5]);
}

// ponizej tylko boilerplate-mapowanie do pythona

static PyMethodDef SickMethods[] = {
    {"fk", forwardKin, METH_VARARGS, "Liczy Kinematyke Prosta (FK) dla 6 katow."},
    {"ik_ccd", inverseKin, METH_VARARGS, "Liczy Kinematyke Odwrotna (IK CCD) do celu X, Y, Z."},
    {NULL, NULL, 0, NULL} 
};

static struct PyModuleDef sickmodule = {
    PyModuleDef_HEAD_INIT,
    "c_kinematix", 
    "Modul do kinematyki robota w C",
    -1,
    SickMethods
};

PyMODINIT_FUNC PyInit_c_kinematix(void) {
    PyObject* m = PyModule_Create(&sickmodule); // Stwórz moduł i przypisz do 'm'
    if (!m) {return NULL;}
    return m;
}