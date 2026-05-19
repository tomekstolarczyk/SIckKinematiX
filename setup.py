import setuptools

module = setuptools.Extension(
    "sickkinematix.c_kinematix", 
    sources=[
        "src/wrapper.c",
        "src/pureMath.c",
        "src/forwardKinematics.c"
    ]
)

setuptools.setup(
    packages=["sickkinematix"], 
    ext_modules=[module],       
)