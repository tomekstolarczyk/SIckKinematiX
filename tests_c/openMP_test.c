#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../src/justMath.h"

#ifdef _OPENMP
#include <omp.h>
#endif

int main() {
    printf("--- RUNNING C CORE TESTS ---\n");

    // 1. Test matematyki 4x4
    Matrix44 id;
    create4x4IdentityMatrix(&id);
    assert(id.data[0] == 1.0);
    assert(id.data[1] == 0.0);
    assert(id.data[15] == 1.0);
    printf("[PASS] Identity Matrix creation\n");

    // 2. Test OpenMP
    #ifdef _OPENMP
        int threads = omp_get_max_threads();
        printf("[INFO] OpenMP is ACTIVE. Max threads: %d\n", threads);
        assert(threads > 0);
    #else
        printf("[WARN] OpenMP is NOT active in this build!\n");
    #endif

    printf("--- ALL C TESTS PASSED ---\n");
    return 0;
}