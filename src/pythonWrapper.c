#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "inverseKinematics.h"
#include "workspaceAnalysis.h"

// numpy c api
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION // to avoid warnings about possible use of old API
#define PY_ARRAY_UNIQUE_SYMBOL SICK_KINEMATIX_API // dla kompilatora - zapobiegamy konfliktowi nazw
#include "numpy/ndarrayobject.h"


// Prawdziwe parametry Modified DH (Craig) dla robota UR5
RobotArm6DoF ramie = {
    // a - przesunięcia wzdłuż osi X (długości głównych kości)
    {0.0, 0.0, -0.425, -0.39225, 0.0, 0.0}, 
    // alpha - skręcenia wokół osi X (w radianach)
    {0.0, M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0},
    // d - przesunięcia wzdłuż osi Z (wysokość bazy i odsunięcia nadgarstka)
    {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823},
    // offsets - ewentualne początkowe offsety kątowe 
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
};

// pakujemy kinematyke prosta
static PyObject* forwardKinWrapped(PyObject* self, PyObject* args)
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

// pakujemy kinematyke odwrotna
static PyObject* inverseKinCCDWrapped(PyObject* self, PyObject* args)
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

// pakujemy workspace analysis
static PyObject* workspaceAnalyzerWrapped(PyObject* self, PyObject* args)
{
    // wypakowujemy
    npy_intp pointsNumber;
    if (!PyArg_ParseTuple(args, "n", &pointsNumber))
        return NULL;
    
    // wymiar tablicy - dla numpy
    npy_intp dims[] = {pointsNumber};

    // z poziomu c tworzymy numpyowe obiekty 
    PyObject* x_array = PyArray_SimpleNew(1, dims, NPY_DOUBLE);
    PyObject* y_array = PyArray_SimpleNew(1, dims, NPY_DOUBLE);
    PyObject* z_array = PyArray_SimpleNew(1, dims, NPY_DOUBLE);

    // jesli ktoras z tablic niepoprawnie sie zainicjalizuje to czyscimy
    if(!x_array || !y_array || !z_array)
    {
        Py_XDECREF(x_array); Py_XDECREF(y_array); Py_XDECREF(z_array);
        return NULL;
    }

    // teraz wyciagamy z tych obiektow wskazniki na tablice danych
    double* x_data = (double*) PyArray_DATA((PyArrayObject*) x_array);
    double* y_data = (double*) PyArray_DATA((PyArrayObject*) y_array);
    double* z_data = (double*) PyArray_DATA((PyArrayObject*) z_array);

    // printf("--> [C DEBUG] Tablice NumPy utworzone, wchodze do workspaceAnalysis...\n");

    // nasz agorytm wypelnia zadane tablice danymi
    workspaceAnalysis(&ramie, pointsNumber, x_data, y_data, z_data);

    // printf("--> [C DEBUG] Wróciłem z workspaceAnalysis. Pakuje do Pythona...\n");

    // pakujemy to i odslyamy wskaznik na to do pythona
    return Py_BuildValue("NNN", x_array, y_array, z_array);
    // N nie O bo nie chcemy zwiekszac licznika referencji
    // kiedy wywolalismy PyArray_SimpleNew, NumPy stworzylo obiekt i od razu ustawilo jego licznik na 1
    // nie ma co go zwiekszac juz
}

// ponizej tylko boilerplate-mapowanie do pythona --------------------------------------------------------------------------------

static PyMethodDef SickMethods[] = {
    {"fk", forwardKinWrapped, METH_VARARGS, "Liczy Kinematyke Prosta (FK) dla 6 katow."},
    {"ik_ccd", inverseKinCCDWrapped, METH_VARARGS, "Liczy Kinematyke Odwrotna (IK CCD) do celu X, Y, Z."},
    {"workspace", workspaceAnalyzerWrapped, METH_VARARGS, "Generuje chmure punktow przestrzeni roboczej"},
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