#ifndef BOIDS_GPU_H
#define BOIDS_GPU_H

#include "boids.cu"

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
    static Boids d;
    static int initialized = 0;

    if (!initialized) {
        cudaMalloc(&d.pos_x, b->count * sizeof(float));
        cudaMalloc(&d.pos_y, b->count * sizeof(float));
        cudaMalloc(&d.vel_x, b->count * sizeof(float));
        cudaMalloc(&d.vel_y, b->count * sizeof(float));
        d.count = b->count;
        initialized = 1;
    }

    cudaMemcpyToSymbol(d_params, p, sizeof(BoidsParams), 0, cudaMemcpyHostToDevice);

    cudaMemcpy(d.pos_x, b->pos_x, b->count * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d.pos_y, b->pos_y, b->count * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d.vel_x, b->vel_x, b->count * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d.vel_y, b->vel_y, b->count * sizeof(float), cudaMemcpyHostToDevice);

    int threads = 128;
    int blocks = (b->count + threads - 1) / threads;

    boids_update_kernel<<<blocks, threads>>>(d, dt);
    cudaDeviceSynchronize();

    cudaMemcpy(b->pos_x, d.pos_x, b->count * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(b->pos_y, d.pos_y, b->count * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(b->vel_x, d.vel_x, b->count * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(b->vel_y, d.vel_y, b->count * sizeof(float), cudaMemcpyDeviceToHost);
}


#endif /* BOIDS_GPU_H */
