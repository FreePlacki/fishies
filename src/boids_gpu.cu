#ifndef BOIDS_GPU_H
#define BOIDS_GPU_H

#include "boids.cu"

#define CUDA_CHECK(call)                                                       \
    do {                                                                       \
        cudaError_t err = (call);                                              \
        if (err != cudaSuccess) {                                              \
            fprintf(stderr, "CUDA error %s:%d: %s\n", __FILE__, __LINE__,      \
                    cudaGetErrorString(err));                                  \
            goto cleanup;                                                      \
        }                                                                      \
    } while (0)

__constant__ BoidsParams d_params;

__global__ void boids_update_kernel(Boids b, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= b.count) return;

    rule_cohesion(&b, i, &d_params, dt);
    rule_separation(&b, i, &d_params, dt);
    rule_alignment(&b, i, &d_params, dt);

    clamp_speed(&b.vel_x[i], &b.vel_y[i], &d_params);

    update_pos(&b, i, dt);
}

void boids_update_gpu(Boids *b, const BoidsParams *p, float dt) {
    static Boids d_boids;
    static int initialized = 0;

    int threads = 128;
    int blocks = (b->count + threads - 1) / threads;

    if (!initialized) {
        CUDA_CHECK(cudaMalloc(&d_boids.pos_x, b->count * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_boids.pos_y, b->count * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_boids.vel_x, b->count * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_boids.vel_y, b->count * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_boids.type, b->count * sizeof(u8)));
        d_boids.count = b->count;
        initialized = 1;
    }

    CUDA_CHECK(cudaMemcpyToSymbol(d_params, p, sizeof(BoidsParams), 0, cudaMemcpyHostToDevice));

    CUDA_CHECK(cudaMemcpy(d_boids.pos_x, b->pos_x, b->count * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_boids.pos_y, b->pos_y, b->count * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_boids.vel_x, b->vel_x, b->count * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_boids.vel_y, b->vel_y, b->count * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_boids.type, b->type, b->count * sizeof(u8), cudaMemcpyHostToDevice));

    boids_update_kernel<<<blocks, threads>>>(d_boids, dt);
    CUDA_CHECK(cudaDeviceSynchronize());

    CUDA_CHECK(cudaMemcpy(b->pos_x, d_boids.pos_x, b->count * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(b->pos_y, d_boids.pos_y, b->count * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(b->vel_x, d_boids.vel_x, b->count * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(b->vel_y, d_boids.vel_y, b->count * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(b->type, d_boids.type, b->count * sizeof(u8), cudaMemcpyDeviceToHost));

cleanup:
    return;
}


#endif /* BOIDS_GPU_H */
