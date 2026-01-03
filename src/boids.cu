#ifndef BOIDS_H
#define BOIDS_H

#include <stdio.h>
#include <stdlib.h>

typedef struct {
    float *pos_x;
    float *pos_y;
    float *vel_x;
    float *vel_y;
    int count;
} Boids;

static float frand() { return (float)rand() / (float)RAND_MAX; }

void boids_init(Boids *boids, int count) {
    boids->pos_x = (float *)malloc(count * sizeof(float));
    boids->pos_y = (float *)malloc(count * sizeof(float));
    boids->vel_x = (float *)malloc(count * sizeof(float));
    boids->vel_y = (float *)malloc(count * sizeof(float));

    if (boids->pos_x == NULL || boids->pos_y == NULL || boids->vel_x == NULL ||
        boids->vel_y == NULL) {
        printf("Failed to allocate boids\n");
        exit(1);
    }

    boids->count = count;

    for (int i = 0; i < count; ++i) {
        boids->pos_x[i] = frand() * 2.0f - 1.0f;
        boids->pos_y[i] = frand() * 2.0f - 1.0f;
        boids->vel_x[i] = 1.5f * (frand() * 2.0f - 1.0f);
        boids->vel_y[i] = 1.5f * (frand() * 2.0f - 1.0f);
    }
}

void boids_free(Boids *boids) {
    if (!boids)
        return;

    if (boids->pos_x)
        free(boids->pos_x);
    if (boids->pos_y)
        free(boids->pos_y);
    if (boids->vel_x)
        free(boids->vel_x);
    if (boids->vel_y)
        free(boids->vel_y);

    boids->count = 0;
}

typedef struct {
    float px, py;
    float vx, vy;
} RenderBoid;

void boids_pack_positions(const Boids *b, RenderBoid *out) {
    for (int i = 0; i < b->count; i++) {
        out[i].px = b->pos_x[i];
        out[i].py = b->pos_y[i];
        out[i].vx = b->vel_x[i];
        out[i].vy = b->vel_y[i];
    }
}

__host__ __device__ __forceinline__ float wrap(float x) {
    if (x < -1.0f)
        return x + 2.0f;
    if (x > 1.0f)
        return x - 2.0f;
    return x;
}

__host__ __device__ __forceinline__ float wrap_delta(float d) {
    if (d > 1.0f)
        d -= 2.0f;
    if (d < -1.0f)
        d += 2.0f;
    return d;
}

__host__ __device__ void update_pos(Boids *b, int i, float dt) {
    b->pos_x[i] = wrap(b->pos_x[i] + b->vel_x[i] * dt);
    b->pos_y[i] = wrap(b->pos_y[i] + b->vel_y[i] * dt);
}

#define COHESION_R 0.10f
#define COHESION_R2 (COHESION_R * COHESION_R)
#define COHESION_STRENGTH 1.0f

__host__ __device__ void rule_cohesion(Boids *b, int i, float dt) {
    float cx = 0.0f;
    float cy = 0.0f;
    int nei = 0;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];
    for (int j = 0; j < b->count; j++) {
        if (i == j)
            continue;
        float dx = wrap_delta(b->pos_x[j] - ix);
        float dy = wrap_delta(b->pos_y[j] - iy);

        if (dx * dx + dy * dy < COHESION_R2) {
            cx += b->pos_x[j];
            cy += b->pos_y[j];
            nei++;
        }
    }

    if (nei == 0)
        return;
    cx /= nei;
    cy /= nei;

    float steer_x = cx - ix;
    float steer_y = cy - iy;

    b->vel_x[i] += steer_x * COHESION_STRENGTH * dt;
    b->vel_y[i] += steer_y * COHESION_STRENGTH * dt;
}

#define SEPARATION_R 0.05f
#define SEPARATION_R2 (SEPARATION_R * SEPARATION_R)
#define SEPARATION_STRENGTH 1.0f

__host__ __device__ void rule_separation(Boids *b, int i, float dt) {
    float steer_x = 0.0f;
    float steer_y = 0.0f;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];

    for (int j = 0; j < b->count; ++j) {
        if (i == j)
            continue;

        float dx = wrap_delta(ix - b->pos_x[j]);
        float dy = wrap_delta(iy - b->pos_y[j]);
        float dist2 = dx * dx + dy * dy;

        if (dist2 > 0.0f && dist2 < SEPARATION_R2) {
            steer_x += dx;
            steer_y += dy;
        }
    }

    b->vel_x[i] += steer_x * SEPARATION_STRENGTH * dt;
    b->vel_y[i] += steer_y * SEPARATION_STRENGTH * dt;
}

#define ALIGNMENT_R 0.15f
#define ALIGNMENT_R2 (ALIGNMENT_R * ALIGNMENT_R)
#define ALIGNMENT_STRENGTH 1.0f

__host__ __device__ void rule_alignment(Boids *b, int i, float dt) {
    float avg_vx = 0.0f;
    float avg_vy = 0.0f;
    int nei = 0;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];

    for (int j = 0; j < b->count; ++j) {
        if (i == j)
            continue;

        float dx = wrap_delta(b->pos_x[j] - ix);
        float dy = wrap_delta(b->pos_y[j] - iy);

        if (dx * dx + dy * dy < ALIGNMENT_R2) {
            avg_vx += b->vel_x[j];
            avg_vy += b->vel_y[j];
            nei++;
        }
    }

    if (nei == 0)
        return;
    avg_vx /= nei;
    avg_vy /= nei;

    b->vel_x[i] += (avg_vx - b->vel_x[i]) * ALIGNMENT_STRENGTH * dt;
    b->vel_y[i] += (avg_vy - b->vel_y[i]) * ALIGNMENT_STRENGTH * dt;
}

#define MIN_SPEED 0.5f
#define MAX_SPEED 3.0f
__device__ __host__ __forceinline__ void clamp_speed(float *vx, float *vy) {
    float v2 = (*vx) * (*vx) + (*vy) * (*vy);
    if (v2 < 1e-6f)
        return;

    float v = sqrtf(v2);

    if (v < MIN_SPEED) {
        float s = MIN_SPEED / v;
        *vx *= s;
        *vy *= s;
    } else if (v > MAX_SPEED) {
        float s = MAX_SPEED / v;
        *vx *= s;
        *vy *= s;
    }
}

void boids_update(Boids *b, float dt) {
    for (int i = 0; i < b->count; i++) {
        rule_cohesion(b, i, dt);
        rule_separation(b, i, dt);
        rule_alignment(b, i, dt);

        clamp_speed(&b->vel_x[i], &b->vel_y[i]);

        update_pos(b, i, dt);
    }
}

#endif /* BOIDS_H */
