#ifndef BOIDS_GPU_H
#define BOIDS_GPU_H

#include "boids.cu"
#include <cub/cub.cuh>

#define CUDA_CHECK(call)                                                       \
    do {                                                                       \
        cudaError_t err = (call);                                              \
        if (err != cudaSuccess) {                                              \
            fprintf(stderr, "CUDA error %s:%d: %s\n", __FILE__, __LINE__,      \
                    cudaGetErrorString(err));                                  \
            goto cleanup;                                                      \
        }                                                                      \
    } while (0)

#define GRID_SIZE 0.1f
#define GRID_MIN -1.0f
#define GRID_MAX 1.0f
#define GRID_DIM ((int)((GRID_MAX - GRID_MIN) / GRID_SIZE))

__device__ __forceinline__ int cell_index(float x, float y) {
    int ix = (int)((x - GRID_MIN) / GRID_SIZE);
    int iy = (int)((y - GRID_MIN) / GRID_SIZE);

    ix = max(0, min(ix, GRID_DIM - 1));
    iy = max(0, min(iy, GRID_DIM - 1));

    return iy * GRID_DIM + ix;
}

__global__ void grid_count_kernel(Boids b, int *cell_counts) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= b.count)
        return;

    int cell = cell_index(b.pos_x[i], b.pos_y[i]);
    atomicAdd(&cell_counts[cell], 1);
}

void compute_cell_offsets(int *counts, int *offsets) {
    static void *temp = NULL;
    static size_t temp_bytes = 0;

    cub::DeviceScan::ExclusiveSum(temp, temp_bytes, counts, offsets,
                                  GRID_DIM * GRID_DIM);

    if (!temp) {
        cudaMalloc(&temp, temp_bytes);
        cub::DeviceScan::ExclusiveSum(temp, temp_bytes, counts, offsets,
                                      GRID_DIM * GRID_DIM);
    }
}

__global__ void grid_fill_kernel(Boids b, const int *cell_offsets,
                                 int *cell_counts, int *cell_indices) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= b.count)
        return;

    int cell = cell_index(b.pos_x[i], b.pos_y[i]);
    int offset = atomicAdd(&cell_counts[cell], 1);

    cell_indices[cell_offsets[cell] + offset] = i;
}

__constant__ BoidsParams d_params;

__device__ __forceinline__ float wrap(float x) {
    if (x < -1.0f)
        return x + 2.0f;
    if (x > 1.0f)
        return x - 2.0f;
    return x;
}

__device__ __forceinline__ float wrap_delta(float d) {
    if (d > 1.0f)
        d -= 2.0f;
    if (d < -1.0f)
        d += 2.0f;
    return d;
}

__device__ __forceinline__ void update_pos(Boids *b, int i, float dt) {
    b->pos_x[i] = wrap(b->pos_x[i] + b->vel_x[i] * dt);
    b->pos_y[i] = wrap(b->pos_y[i] + b->vel_y[i] * dt);
}

__device__ __forceinline__ void rule_cohesion(Boids *b, int i, int start, int count,
                              int *cell_indices, float dt) {
    const u8 ty = b->type[i];
    const float r2 =
        d_params.type[ty].cohesion_r * d_params.type[ty].cohesion_r;

    float cx = 0.0f;
    float cy = 0.0f;
    int nei = 0;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];
    for (int k = 0; k < count; ++k) {
        int j = cell_indices[start + k];
        if (j == i || b->type[j] != ty)
            continue;
        float dx = wrap_delta(b->pos_x[j] - ix);
        float dy = wrap_delta(b->pos_y[j] - iy);

        if (dx * dx + dy * dy < r2) {
            cx += ix + dx;
            cy += iy + dy;
            nei++;
        }
    }

    if (nei == 0)
        return;
    cx /= nei;
    cy /= nei;

    float steer_x = cx - ix;
    float steer_y = cy - iy;

    float strength = d_params.type[ty].cohesion_strength;
    if (d_params.cursor_state == RMB &&
        (d_params.cursor_x - ix) * (d_params.cursor_x - ix) +
                (d_params.cursor_y - iy) * (d_params.cursor_y - iy) <
            d_params.cursor_r * d_params.cursor_r)
        strength *= -10.0f;
    b->vel_x[i] += steer_x * strength * dt;
    b->vel_y[i] += steer_y * strength * dt;
}

__device__ __forceinline__ void rule_separation(Boids *b, int i, int start, int count,
                                int *cell_indices, float dt) {
    const u8 ty = b->type[i];
    const float r2 =
        d_params.type[ty].separation_r * d_params.type[ty].separation_r;

    float steer_x = 0.0f;
    float steer_y = 0.0f;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];
    for (int k = 0; k < count; ++k) {
        int j = cell_indices[start + k];
        if (j == i)
            continue;

        float dx = wrap_delta(ix - b->pos_x[j]);
        float dy = wrap_delta(iy - b->pos_y[j]);
        float dist2 = dx * dx + dy * dy;

        float inv = 0.001f / dist2;
        if (inv > 100.0f)
            inv = 100.0f;
        if (dist2 > 0.0f && dist2 < r2) {
            steer_x += dx * inv;
            steer_y += dy * inv;
        }
    }

    float strength = d_params.type[ty].separation_strength;
    b->vel_x[i] += steer_x * strength * dt;
    b->vel_y[i] += steer_y * strength * dt;
}

__device__ __forceinline__ void rule_alignment(Boids *b, int i, int start, int count,
                               int *cell_indices, float dt) {
    const u8 ty = b->type[i];
    const float r2 =
        d_params.type[ty].alignment_r * d_params.type[ty].alignment_r;

    float avg_vx = 0.0f;
    float avg_vy = 0.0f;
    int nei = 0;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];
    for (int k = 0; k < count; ++k) {
        int j = cell_indices[start + k];
        if (j == i || b->type[j] != ty)
            continue;

        float dx = wrap_delta(b->pos_x[j] - ix);
        float dy = wrap_delta(b->pos_y[j] - iy);

        if (dx * dx + dy * dy < r2) {
            avg_vx += b->vel_x[j];
            avg_vy += b->vel_y[j];
            nei++;
        }
    }

    if (nei == 0)
        return;
    avg_vx /= nei;
    avg_vy /= nei;

    float strength = d_params.type[ty].alignment_strength;
    b->vel_x[i] += (avg_vx - b->vel_x[i]) * strength * dt;
    b->vel_y[i] += (avg_vy - b->vel_y[i]) * strength * dt;
}

__device__ __forceinline__ void rule_cursor(Boids *b, int i, float dt) {
    if (d_params.cursor_state == NONE)
        return;

    const float r2 = d_params.cursor_r * d_params.cursor_r;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];
    float cx = d_params.cursor_x;
    float cy = d_params.cursor_y;

    float dx = wrap_delta(cx - ix);
    float dy = wrap_delta(cy - iy);

    if (dx * dx + dy * dy > r2)
        return;

    float strength = d_params.cursor_strength;
    if (d_params.cursor_state == RMB)
        strength *= -5.0f;
    b->vel_x[i] += (cx - ix) * strength * dt;
    b->vel_y[i] += (cy - iy) * strength * dt;
}

__device__ __forceinline__ void clamp_speed(Boids *b, int i) {
    float *vx = &b->vel_x[i];
    float *vy = &b->vel_y[i];
    float v2 = (*vx) * (*vx) + (*vy) * (*vy);
    if (v2 < 1e-6f)
        return;

    float v = sqrtf(v2);

    float min = d_params.type[b->type[i]].min_speed;
    float max = d_params.type[b->type[i]].min_speed;
    if (v < min) {
        float s = min / v;
        *vx *= s;
        *vy *= s;
    } else if (v > max) {
        float s = max / v;
        *vx *= s;
        *vy *= s;
    }
}

__global__ void boids_update_kernel(Boids b, int *d_cell_offsets,
                                    int *d_cell_counts, int *d_cell_indices,
                                    float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= b.count)
        return;

    int cx = (int)((b.pos_x[i] - GRID_MIN) / GRID_SIZE);
    int cy = (int)((b.pos_y[i] - GRID_MIN) / GRID_SIZE);

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int nx = cx + dx;
            int ny = cy + dy;
            if (nx < 0 || ny < 0 || nx >= GRID_DIM || ny >= GRID_DIM)
                continue;

            int cell = ny * GRID_DIM + nx;
            int start = d_cell_offsets[cell];
            int count = d_cell_counts[cell];

            rule_cohesion(&b, i, start, count, d_cell_indices, dt);
            rule_separation(&b, i, start, count, d_cell_indices, dt);
            rule_alignment(&b, i, start, count, d_cell_indices, dt);
        }
    }

    rule_cursor(&b, i, dt);
    clamp_speed(&b, i);
    update_pos(&b, i, dt);
}

void boids_update_gpu(Boids *b, const BoidsParams *p, float dt) {
    static Boids d_boids;
    static int initialized = 0;

    static int *d_cell_counts = NULL;
    static int *d_cell_offsets = NULL;
    static int *d_cell_indices = NULL;

    int threads = 128;
    int blocks = (b->count + threads - 1) / threads;

    if (!initialized) {
        d_boids.count = b->count;
        CUDA_CHECK(cudaMalloc(&d_boids.pos_x, d_boids.count * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_boids.pos_y, d_boids.count * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_boids.vel_x, d_boids.count * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_boids.vel_y, d_boids.count * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_boids.type, d_boids.count * sizeof(u8)));

        CUDA_CHECK(
            cudaMalloc(&d_cell_counts, GRID_DIM * GRID_DIM * sizeof(int)));
        CUDA_CHECK(
            cudaMalloc(&d_cell_offsets, GRID_DIM * GRID_DIM * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&d_cell_indices, d_boids.count * sizeof(int)));

        initialized = 1;
    }

    CUDA_CHECK(cudaMemcpyToSymbol(d_params, p, sizeof(BoidsParams), 0,
                                  cudaMemcpyHostToDevice));

    CUDA_CHECK(cudaMemcpy(d_boids.pos_x, b->pos_x, b->count * sizeof(float),
                          cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_boids.pos_y, b->pos_y, b->count * sizeof(float),
                          cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_boids.vel_x, b->vel_x, b->count * sizeof(float),
                          cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_boids.vel_y, b->vel_y, b->count * sizeof(float),
                          cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_boids.type, b->type, b->count * sizeof(u8),
                          cudaMemcpyHostToDevice));

    CUDA_CHECK(cudaMemset(d_cell_counts, 0, GRID_DIM * GRID_DIM * sizeof(int)));
    grid_count_kernel<<<blocks, threads>>>(d_boids, d_cell_counts);
    compute_cell_offsets(d_cell_counts, d_cell_offsets);
    CUDA_CHECK(cudaMemset(d_cell_counts, 0, GRID_DIM * GRID_DIM * sizeof(int)));
    grid_fill_kernel<<<blocks, threads>>>(d_boids, d_cell_offsets,
                                          d_cell_counts, d_cell_indices);

    boids_update_kernel<<<blocks, threads>>>(d_boids, d_cell_offsets,
                                             d_cell_counts, d_cell_indices, dt);
    CUDA_CHECK(cudaDeviceSynchronize());

    CUDA_CHECK(cudaMemcpy(b->pos_x, d_boids.pos_x, b->count * sizeof(float),
                          cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(b->pos_y, d_boids.pos_y, b->count * sizeof(float),
                          cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(b->vel_x, d_boids.vel_x, b->count * sizeof(float),
                          cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(b->vel_y, d_boids.vel_y, b->count * sizeof(float),
                          cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(b->type, d_boids.type, b->count * sizeof(u8),
                          cudaMemcpyDeviceToHost));

cleanup:
    return;
}

#endif /* BOIDS_GPU_H */
