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
        boids->vel_x[i] = 0.01f * (frand() * 2.0f - 1.0f);
        boids->vel_y[i] = 0.01f * (frand() * 2.0f - 1.0f);
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
    float px, py;   // position (NDC)
    float vx, vy;   // velocity
} RenderBoid;

void boids_pack_positions(const Boids *b, RenderBoid *out) {
    for (int i = 0; i < b->count; i++) {
        out[i].px = b->pos_x[i];
        out[i].py = b->pos_y[i];
        out[i].vx = b->vel_x[i];
        out[i].vy = b->vel_y[i];
    }
}
