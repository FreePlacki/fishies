#include "assert.h"
#include <cuda_runtime.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#endif

void usage(char *pname) {
    fprintf(stderr, "Usage\t%s\n", pname);
    exit(1);
}

int main(int argc, char **argv) {
    srand(time(0));

    cudaError_t cudaStatus = cudaSetDevice(0);
    int noCudaDevice = cudaStatus != cudaSuccess;
    if (noCudaDevice) {
        printf("cudaSetDevice failed: %d (%s)\n", cudaStatus,
               cudaGetErrorString(cudaStatus));
    }

    char *pname = argv[0];
    if (argc != 1)
        usage(pname);

    return 0;
}
