import setuptools
import numpy

module = setuptools.Extension(
    "sickkinematix.c_kinematix", 
    sources=[
        "src/pythonWrapper.c",
        "src/robot.c",
        "src/justMath.c",
        "src/forwardKinematics.c",
        "src/inverseKinematics.c",
        "src/workspaceAnalysis.c"
    ],
    include_dirs=[numpy.get_include()] 
)

setuptools.setup(
    packages=["sickkinematix"], 
    ext_modules=[module],       
)