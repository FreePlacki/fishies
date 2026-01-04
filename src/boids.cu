#ifndef BOIDS_H
#define BOIDS_H

#include "params.cu"
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    float *pos_x;
    float *pos_y;
    float *vel_x;
    float *vel_y;
    u8 *type;
    int count;
} Boids;

static float frand() { return (float)rand() / (float)RAND_MAX; }

void boids_init(Boids *boids, const InitialConfig *cfg) {
    int total = 0;
    for (int i = 0; i < cfg->type_count; i++)
        total += cfg->boids_per_type[i];

    boids->pos_x = (float *)malloc(total * sizeof(float));
    boids->pos_y = (float *)malloc(total * sizeof(float));
    boids->vel_x = (float *)malloc(total * sizeof(float));
    boids->vel_y = (float *)malloc(total * sizeof(float));
    boids->type = (u8 *)malloc(total * sizeof(u8));

    if (boids->pos_x == NULL || boids->pos_y == NULL || boids->vel_x == NULL ||
        boids->vel_y == NULL) {
        printf("Failed to allocate boids\n");
        exit(1);
    }

    boids->count = total;

    int curr_type = 0;
    int curr_type_count = 0;
    for (int i = 0; i < total; ++i) {
        boids->pos_x[i] = frand() * 2.0f - 1.0f;
        boids->pos_y[i] = frand() * 2.0f - 1.0f;
        boids->vel_x[i] = 1.5f * (frand() * 2.0f - 1.0f);
        boids->vel_y[i] = 1.5f * (frand() * 2.0f - 1.0f);
        boids->type[i] = curr_type;
        curr_type_count++;
        if (curr_type_count == cfg->boids_per_type[curr_type]) {
            curr_type++;
            curr_type_count = 0;
        }
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
    u8 type;
} RenderBoid;

void boids_pack_positions(const Boids *b, RenderBoid *out) {
    for (int i = 0; i < b->count; i++) {
        out[i].px = b->pos_x[i];
        out[i].py = b->pos_y[i];
        out[i].vx = b->vel_x[i];
        out[i].vy = b->vel_y[i];
        out[i].type = b->type[i];
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

__host__ __device__ void rule_cohesion(Boids *b, int i, const BoidsParams *p,
                                       float dt) {
    float r2 = p->cohesion_r * p->cohesion_r;

    float cx = 0.0f;
    float cy = 0.0f;
    int nei = 0;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];
    u8 ty = b->type[i];
    for (int j = 0; j < b->count; j++) {
        if (i == j || b->type[j] != ty)
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

    float strength = p->cohesion_strength;
    if (p->cursor_state == RMB &&
        (p->cursor_x - ix) * (p->cursor_x - ix) +
                (p->cursor_y - iy) * (p->cursor_y - iy) <
            p->cursor_r * p->cursor_r)
        strength *= -10.0f;
    b->vel_x[i] += steer_x * strength * dt;
    b->vel_y[i] += steer_y * strength * dt;
}

__host__ __device__ void rule_separation(Boids *b, int i, const BoidsParams *p,
                                         float dt) {
    const float r2 = p->separation_r * p->separation_r;

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

        float inv = 0.001f / dist2;
        if (inv > 100.0f)
            inv = 100.0f;
        if (dist2 > 0.0f && dist2 < r2) {
            steer_x += dx * inv;
            steer_y += dy * inv;
        }
    }

    b->vel_x[i] += steer_x * p->separation_strength * dt;
    b->vel_y[i] += steer_y * p->separation_strength * dt;
}

__host__ __device__ void rule_alignment(Boids *b, int i, const BoidsParams *p,
                                        float dt) {
    const float r2 = p->alignment_r * p->alignment_r;

    float avg_vx = 0.0f;
    float avg_vy = 0.0f;
    int nei = 0;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];
    u8 ty = b->type[i];
    for (int j = 0; j < b->count; ++j) {
        if (i == j || b->type[j] != ty)
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

    b->vel_x[i] += (avg_vx - b->vel_x[i]) * p->alignment_strength * dt;
    b->vel_y[i] += (avg_vy - b->vel_y[i]) * p->alignment_strength * dt;
}

__host__ __device__ void rule_cursor(Boids *b, int i, const BoidsParams *p,
                                     float dt) {
    if (p->cursor_state == NONE)
        return;

    const float r2 = p->cursor_r * p->cursor_r;

    float ix = b->pos_x[i];
    float iy = b->pos_y[i];
    float cx = p->cursor_x;
    float cy = p->cursor_y;

    float dx = wrap_delta(cx - ix);
    float dy = wrap_delta(cy - iy);

    if (dx * dx + dy * dy > r2)
        return;

    float strength = p->cursor_strength;
    if (p->cursor_state == RMB)
        strength *= -5.0f;
    b->vel_x[i] += (cx - ix) * strength * dt;
    b->vel_y[i] += (cy - iy) * strength * dt;
}

__device__ __host__ void clamp_speed(float *vx, float *vy,
                                     const BoidsParams *p) {
    float v2 = (*vx) * (*vx) + (*vy) * (*vy);
    if (v2 < 1e-6f)
        return;

    float v = sqrtf(v2);

    if (v < p->min_speed) {
        float s = p->min_speed / v;
        *vx *= s;
        *vy *= s;
    } else if (v > p->max_speed) {
        float s = p->max_speed / v;
        *vx *= s;
        *vy *= s;
    }
}

void boids_update(Boids *b, const BoidsParams *p, float dt) {
    for (int i = 0; i < b->count; i++) {
        rule_cohesion(b, i, p, dt);
        rule_separation(b, i, p, dt);
        rule_alignment(b, i, p, dt);
        rule_cursor(b, i, p, dt);

        clamp_speed(&b->vel_x[i], &b->vel_y[i], p);

        update_pos(b, i, dt);
    }
}

#endif /* BOIDS_H */
