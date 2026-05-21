import setuptools

module = setuptools.Extension(
    "sickkinematix.c_kinematix", 
    sources=[
        "src/pythonWrapper.c",
        "src/pureMath.c",
        "src/forwardKinematics.c",
        "src/inverseKinematics.c"
    ]
)

setuptools.setup(
    packages=["sickkinematix"], 
    ext_modules=[module],       
)