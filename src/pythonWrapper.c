#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "pureMath.h"
#include "forwardKinematics.h"
#include "inverseKinematics.h"
#include "../src/forwardKinematics.h"

RobotArm6DoF ramie = {
    {0, 0, -0.4, -0.2, 0, 0}, // Podstaw te same wartości co w tests_c/ramieRobota.h
    {0, M_PI/2.0, 0, 0, -M_PI/2.0, M_PI/2.0},
    {0.6, 0, 0, 0.1, 0, 0},
    {M_PI/2.0, -M_PI/2.0, 0, M_PI/2.0, 0, M_PI/2.0}
};

// laczymy kinematyke prosta z c z pythonem

static PyObject* forwardKin(PyObject* self, PyObject* args)
{
    // 1. ROZPAKOWANIE z Pyhona
    double t1, t2, t3, t4, t5, t6;
    if (!PyArg_ParseTuple(args, "dddddd", &t1, &t2, &t3, &t4, &t5, &t6))
        return NULL;

    // 2. liczymy w c

    double tetas[] = {t1,t2,t3,t4,t5,t6};
    Matrix44 results[6];

    forwardKinematics(&ramie, tetas, results); 

    // 3. pakujemy wyniki spowrotem do pythona

    PyObject* lista = PyList_New(7);

    PyObject* baza = Py_BuildValue("ddd", 0.0, 0.0, 0.0);
    PyList_SetItem(lista, 0, baza);

    for(size_t i = 0; i<6; i++)
    {
        double x = results[i].data[3];
        double y = results[i].data[7];
        double z = results[i].data[11];
        PyObject* wektor = Py_BuildValue("ddd", x, y, z);

        PyList_SetItem(lista, i+1, wektor);
    }

    return lista;
}

// teraz dla kinematyki odwrotnej ccd

static PyObject* inverseKin(PyObject* self, PyObject* args)
{
    // 1. rozpakowanie 
    // "ddd|dddddd" -> target + katy startowe (opcjonalnie)
    double tx, ty, tz, t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t5 = 0.0, t6 = 0.0;
    if (!PyArg_ParseTuple(args, "ddd|dddddd", &tx, &ty, &tz, &t1, &t2, &t3, &t4, &t5, &t6))
        return NULL;
    
    // 2. przeliczamy, start z pozycji zerowej
    Vector3D target = {tx, ty, tz};

    double thetas[] = {t1, t2, t3, t4, t5, t6};
    inverseKinematicsCCD(&ramie, thetas, &target);

    // 3. pakujemy
    return Py_BuildValue("dddddd", thetas[0], thetas[1], thetas[2]
    , thetas[3], thetas[4], thetas[5]);
}

// ponizej tylko boilerplate-mapowanie do pythona --------------------------------------------------------------------------------

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