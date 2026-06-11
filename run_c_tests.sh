#!/bin/bash
echo "====================================="
echo " COMPILING AND RUNNING C TESTS"
echo "====================================="

mkdir -p tests_c/.outs

# Lista dokładnych nazw Twoich plików
TESTS=(
    "forwardKinTest"
    "inverseKinCCDtest"
    "inverseKinDLStest"
    "jacobianTest"
    "workspaceAnalysisTest"
    "yoshikawaTest"
    "openMPTest"
)

# Łapiemy kod C omijając wrappera Pythona
SRC_FILES=$(find src -name "*.c" ! -name "pythonWrapper.c")

for test_name in "${TESTS[@]}"; do
    # Kompilacja
    gcc tests_c/${test_name}.c $SRC_FILES -o tests_c/.outs/${test_name} -lm -I./src
    
    if [ $? -eq 0 ]; then
        ./tests_c/.outs/${test_name}
    else
        echo "[FAIL] Nie udało się skompilować: ${test_name}.c"
        exit 1
    fi
done

echo "====================================="
echo " ALL TESTS PASSED SUCCESSFULLY! "
echo "====================================="