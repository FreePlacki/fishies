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
        boids->vel_x[i] = 0.1f * (frand() * 2.0f - 1.0f);
        boids->vel_y[i] = 0.1f * (frand() * 2.0f - 1.0f);
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

static void update_pos(Boids *b, float dt) {
    for (int i = 0; i < b->count; i++) {
        b->pos_x[i] += b->vel_x[i] * dt;
        b->pos_y[i] += b->vel_y[i] * dt;
    }
}

#define COHESION_R 0.15f
#define COHESION_R2 (COHESION_R * COHESION_R)
#define COHESION_STRENGTH 2.0f

static void rule_cohesion(Boids *b, float dt) {
    for (int i = 0; i < b->count; i++) {
        float cx = 0.0f;
        float cy = 0.0f;
        int nei = 0;

        float ix = b->pos_x[i];
        float iy = b->pos_y[i];
        for (int j = 0; j < b->count; j++) {
            if (i == j)
                continue;
            float dx = b->pos_x[j] - ix;
            float dy = b->pos_y[j] - iy;

            if (dx * dx + dy * dy < COHESION_R2) {
                cx += b->pos_x[j];
                cy += b->pos_y[j];
                nei++;
            }
        }

        if (nei == 0)
            continue;
        cx /= nei;
        cy /= nei;

        float steer_x = cx - ix;
        float steer_y = cy - iy;

        b->vel_x[i] += steer_x * COHESION_STRENGTH * dt;
        b->vel_y[i] += steer_y * COHESION_STRENGTH * dt;
    }
}

void boids_update(Boids *b, float dt) {
    rule_cohesion(b, dt);
    update_pos(b, dt);
}
