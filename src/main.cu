#include "assert.h"
#include <GLFW/glfw3.h>
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

    if (!glfwInit()) {
        printf("GLFW init failed");
        exit(1);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window =
        glfwCreateWindow(800, 600, "Fishies", NULL, NULL);

    if (!window) {
        fprintf(stderr, "Failed to create window\n");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
