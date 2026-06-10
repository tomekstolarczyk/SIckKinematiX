import setuptools
import numpy
import sys

# Ustawianie flag w zależności od systemu 
compile_args = []
link_args = []

if sys.platform.startswith("win"):
    compile_args = ["/openmp"]
elif sys.platform.startswith("linux"):
    compile_args = ["-fopenmp"]
    link_args = ["-lgomp"]
elif sys.platform == "darwin":
    compile_args = ["-Xpreprocessor", "-fopenmp"]
    link_args = ["-lomp"]

module = setuptools.Extension(
    "sickkinematix.c_kinematix", 
    sources=[
        "src/pythonWrapper.c",
        "src/robot.c",
        "src/justMath.c",
        "src/forwardKinematics.c",
        "src/inverseKinematics.c",
        "src/workspaceAnalysis.c",
        "src/jacobian.c"
    ],
    include_dirs=[numpy.get_include()],
    extra_compile_args=compile_args,
    extra_link_args=link_args 
)

setuptools.setup(
    packages=["sickkinematix"], 
    ext_modules=[module],       
)