#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "inverseKinematics.h"
#include "workspaceAnalysis.h"

// numpy c api
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION // to avoid warnings about possible use of old API
#define PY_ARRAY_UNIQUE_SYMBOL SICK_KINEMATIX_API // dla kompilatora - zapobiegamy konfliktowi nazw
#include "numpy/ndarrayobject.h"

static void robotArmDestructor(PyObject* capsule)
{
    // Rzutujemy na nasz wlasciwy typ struktury C
    RobotArm6DoF* ramie = (RobotArm6DoF*)PyCapsule_GetPointer(capsule, "RobotArm");
    if(ramie!=NULL){free(ramie);}
}

static PyObject* buildRobotArmWrapped(PyObject* self, PyObject* args)
{
    PyObject* aPy, *alphaPy, *dPy, *offsetsPy;
    if (!PyArg_ParseTuple(args, "OOOO", &aPy, &alphaPy, &dPy, &offsetsPy)) {return NULL;}

    // rozpakowujemy kazda liste i budujemy robota
    RobotArm6DoF* ramie = malloc(sizeof(RobotArm6DoF));

    PyObject* pomocnicza[] = {aPy, alphaPy, dPy, offsetsPy};
    double* tablicaDocelowa[] = {ramie->a, ramie->alpha, ramie->d, ramie->offsets};
    for(size_t j = 0; j<4; j++)
    {
        PyObject* obiekt = pomocnicza[j];
        for(size_t i = 0; i<6; i++)
        {
            PyObject* element = PySequence_GetItem(obiekt, i);
            double value = PyFloat_AsDouble(element);
            tablicaDocelowa[j][i] = value;
            // PySequence_GetItem tworzy nowa referencje do obiektu:
            Py_DECREF(element);
        }
    }
    return PyCapsule_New(ramie, "RobotArm", robotArmDestructor);
}

// pakujemy kinematyke prosta
static PyObject* forwardKinWrapped(PyObject* self, PyObject* args)
{
    // 1. ROZPAKOWANIE z Pyhona
    double t1, t2, t3, t4, t5, t6;
    PyObject* robotCapsule;
    if (!PyArg_ParseTuple(args, "Odddddd", &robotCapsule, &t1, &t2, &t3, &t4, &t5, &t6)) {return NULL;}

    RobotArm6DoF* ramie = (RobotArm6DoF*)PyCapsule_GetPointer(robotCapsule, "RobotArm");
    
    // 2. liczymy w c
    double tetas[] = {t1,t2,t3,t4,t5,t6};
    Matrix44 results[6];

    forwardKinematics(ramie, tetas, results); 

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

// pakujemy kinematyke odwrotna
static PyObject* inverseKinCCDWrapped(PyObject* self, PyObject* args)
{
    // 1. rozpakowanie 
    // "ddd|dddddd" -> target + katy startowe (opcjonalnie)
    double tx, ty, tz, t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t5 = 0.0, t6 = 0.0;
    PyObject* robotCapsule;
    if (!PyArg_ParseTuple(args, "Oddd|dddddd", &robotCapsule, &tx, &ty, &tz, &t1, &t2, &t3, &t4, &t5, &t6)) {return NULL;}
    
    RobotArm6DoF* ramie = (RobotArm6DoF*)PyCapsule_GetPointer(robotCapsule, "RobotArm");

    // 2. przeliczamy, start z pozycji zerowej
    Vector3D target = {tx, ty, tz};

    double thetas[] = {t1, t2, t3, t4, t5, t6};
    inverseKinematicsCCD(ramie, thetas, &target);

    // 3. pakujemy
    return Py_BuildValue("dddddd", thetas[0], thetas[1], thetas[2]
    , thetas[3], thetas[4], thetas[5]);
}

// pakujemy workspace analysis
static PyObject* workspaceAnalyzerWrapped(PyObject* self, PyObject* args)
{
    // wypakowujemy
    npy_intp pointsNumber;
    PyObject* robotCapsule;
    if (!PyArg_ParseTuple(args, "On", &robotCapsule, &pointsNumber)) {return NULL;}
    
    RobotArm6DoF* ramie = (RobotArm6DoF*)PyCapsule_GetPointer(robotCapsule, "RobotArm");

    // wymiar tablicy - dla numpy
    npy_intp dims[] = {pointsNumber};

    // z poziomu c tworzymy numpyowe obiekty 
    PyObject* x_array = PyArray_SimpleNew(1, dims, NPY_DOUBLE);
    PyObject* y_array = PyArray_SimpleNew(1, dims, NPY_DOUBLE);
    PyObject* z_array = PyArray_SimpleNew(1, dims, NPY_DOUBLE);

    // jesli ktoras z tablic niepoprawnie sie zainicjalizuje to czyscimy
    if(!x_array || !y_array || !z_array) {Py_XDECREF(x_array); Py_XDECREF(y_array); Py_XDECREF(z_array); return NULL;}

    // teraz wyciagamy z tych obiektow wskazniki na tablice danych
    double* x_data = (double*) PyArray_DATA((PyArrayObject*) x_array);
    double* y_data = (double*) PyArray_DATA((PyArrayObject*) y_array);
    double* z_data = (double*) PyArray_DATA((PyArrayObject*) z_array);

    // printf("--> [C DEBUG] Tablice NumPy utworzone, wchodze do workspaceAnalysis...\n");

    // nasz agorytm wypelnia zadane tablice danymi
    workspaceAnalysis(ramie, pointsNumber, x_data, y_data, z_data);

    // printf("--> [C DEBUG] Wróciłem z workspaceAnalysis. Pakuje do Pythona...\n");

    // pakujemy to i odslyamy wskaznik na to do pythona
    return Py_BuildValue("NNN", x_array, y_array, z_array);
    // N nie O bo nie chcemy zwiekszac licznika referencji
    // kiedy wywolalismy PyArray_SimpleNew, NumPy stworzylo obiekt i od razu ustawilo jego licznik na 1
    // nie ma co go zwiekszac juz
}

// pakujemy inverse kinematics DLS
static PyObject* inverseKinDLSWrapped(PyObject* self, PyObject* args)
{
    // 1 rozpakowujemy
    PyObject* robotCapsule;
    PyObject* targetPoseList;
    double t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t5 = 0.0, t6 = 0.0;
    if (!PyArg_ParseTuple(args, "OO|dddddd", &robotCapsule, &targetPoseList, &t1, &t2, &t3, &t4, &t5, &t6)) 
    {return NULL;} 

    RobotArm6DoF* ramie = (RobotArm6DoF*)PyCapsule_GetPointer(robotCapsule, "RobotArm");
    double thetas[] = {t1, t2, t3, t4, t5, t6};

    // konwersja pythonowej listy na nasza strukture Matrix44
    Matrix44 targetPoseMatrix;
    for(size_t i = 0; i< 16; i++)
    {
        PyObject* item = PySequence_GetItem(targetPoseList, i);
        targetPoseMatrix.data[i] = PyFloat_AsDouble(item);
        Py_DECREF(item);
    }

    // 2 obliczenia
    inverseKinematicsDLS(ramie, thetas, &targetPoseMatrix);

    // 3 pakujemy
    return Py_BuildValue("dddddd", thetas[0], thetas[1], thetas[2], thetas[3], thetas[4], thetas[5]);
}

// pakujemy yoshikawe
static PyObject* yoshikawaWrapped(PyObject* self, PyObject* args)
{
    // 1 rozpakowujemy
    PyObject* robotCapsule;
    double t1, t2, t3, t4, t5, t6;
    if (!PyArg_ParseTuple(args, "Odddddd", &robotCapsule, &t1, &t2, &t3, &t4, &t5, &t6)) {return NULL;} 

    double thetas[] = {t1, t2, t3, t4, t5, t6}; Matrix66 jacobian;
    RobotArm6DoF* ramie = (RobotArm6DoF*)PyCapsule_GetPointer(robotCapsule, "RobotArm");

    // 2 obliczenia
    calculateJacobian(ramie, thetas, &jacobian);
    double yoshikawa = calculateManipulabilityYoshikawaIndex(&jacobian);

    // 3 pakujemy
    return PyFloat_FromDouble(yoshikawa);
}

// ponizej tylko boilerplate-mapowanie do pythona --------------------------------------------------------------------------------

static PyMethodDef SickMethods[] = {
    // {"nazwa_w_pythonie", wskaznik_na_funkcje_c, FLAGA, "docstring"}
    {"build_robot", buildRobotArmWrapped, METH_VARARGS, "Buduje robota na podstawie parametrów MDH."},
    {"fk", forwardKinWrapped, METH_VARARGS, "Liczy Kinematyke Prosta (FK) dla 6 katow."},
    {"ik_ccd", inverseKinCCDWrapped, METH_VARARGS, "Liczy Kinematyke Odwrotna (IK CCD) do celu X, Y, Z."},
    {"workspace", workspaceAnalyzerWrapped, METH_VARARGS, "Generuje chmure punktow przestrzeni roboczej"},
    {"ik_dls", inverseKinDLSWrapped, METH_VARARGS, "Liczy Kinematyke Odwrotna (IK DLS) do celu X, Y, Z i konkretnej rotacji."},
    {"yoshikawa", yoshikawaWrapped, METH_VARARGS, "Liczy Manipulability Index Yoshikawy."},
    
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
    import_array();
    PyObject* m = PyModule_Create(&sickmodule); // Stwórz moduł i przypisz do 'm'
    if (!m) {return NULL;}
    return m;
}