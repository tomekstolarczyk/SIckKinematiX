#include <stdio.h>
#include <assert.h>
#ifdef _OPENMP
#include <omp.h>
#endif

int main() {
    #ifdef _OPENMP
        int threads = omp_get_max_threads();
        assert(threads > 0);
        printf("[PASS] openMP_test (OpenMP ENABLED: %d threads)\n", threads);
    #else
        printf("[PASS] openMP_test (OpenMP is natively disabled/bypassed)\n");
    #endif
    return 0;
}