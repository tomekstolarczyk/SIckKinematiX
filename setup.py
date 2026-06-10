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
    compile_args = ["-Xpreprocessor", "-fopenmp", "-I/opt/homebrew/include"]
    link_args = ["-L/opt/homebrew/lib", "-lomp"]

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
    # Dodajemy ścieżki inkludów i bibliotek jawnie dla Maca
    include_dirs=[numpy.get_include()] + (["/opt/homebrew/include"] if sys.platform == "darwin" else []),
    library_dirs=(["/opt/homebrew/lib"] if sys.platform == "darwin" else []),
    extra_compile_args=compile_args,
    extra_link_args=link_args
)

setuptools.setup(
    packages=["sickkinematix"], 
    ext_modules=[module],       
)