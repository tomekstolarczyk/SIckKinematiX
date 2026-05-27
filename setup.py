import setuptools
import numpy

module = setuptools.Extension(
    "sickkinematix.c_kinematix", 
    sources=[
        "src/pythonWrapper.c",
        "src/robotArm.c",
        "src/pureMath.c",
        "src/forwardKinematics.c",
        "src/inverseKinematics.c",
        "src/workspaceAnalysis.c"
    ],
    include_dirs=[numpy.get_include()] # bez tego C nie znajdzie ndarrayobject.h
)

setuptools.setup(
    packages=["sickkinematix"], 
    ext_modules=[module],       
)