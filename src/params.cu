#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>
#include <stdio.h>

typedef uint8_t u8;
typedef uint32_t u32;

typedef enum {
    LMB,
    RMB,
    NONE,
} CursorState;

#define MAX_TYPES 32

typedef struct {
    float cohesion_r;
    float cohesion_strength;
    float separation_r;
    float separation_strength;
    float alignment_r;
    float alignment_strength;

    float min_speed;
    float max_speed;
} BoidTypeParams;

typedef struct {
    CursorState cursor_state;
    float cursor_x;
    float cursor_y;
    float cursor_r;
    float cursor_strength; // + attract, - repel

    BoidTypeParams type[MAX_TYPES];
    u8 type_count;
} BoidsParams;

typedef struct {
    u8 type_count;
    u32 boids_per_type[MAX_TYPES];
} InitialConfig;

static char *trim(char *s) {
    while (isspace(*s))
        s++;
    if (*s == 0)
        return s;

    char *end = s + strlen(s) - 1;
    while (end > s && isspace(*end))
        end--;
    end[1] = '\0';

    return s;
}

static int parse_boids_line(const char *value, InitialConfig *cfg) {
    const char *p = value;
    char *end;

    while (1) {
        long v = strtol(p, &end, 10);
        if (p == end)
            break;

        if (cfg->type_count == MAX_TYPES) {
            return 0;
        }

        cfg->boids_per_type[cfg->type_count++] = (u32)v;
        p = end;
    }

    return cfg->type_count > 0;
}

void params_default(BoidsParams *p, int types_count) {
    BoidTypeParams params;
    params.cohesion_r = 0.03f;
    params.cohesion_strength = 1.0f;
    params.separation_r = 0.01f;
    params.separation_strength = 0.5f;
    params.alignment_r = 0.04f;
    params.alignment_strength = 1.0f;
    params.min_speed = 0.20f;
    params.max_speed = 3.0f;
    for (int i = 0; i < types_count; i++) {
        p->type[i] = params;
    }
    p->type_count = types_count;

    p->cursor_r = 0.2f;
    p->cursor_strength = 3.0f;
}

int config_parse(const char *fname, InitialConfig *cfg) {
    cfg->type_count = 1;
    cfg->boids_per_type[0] = 1000;

    FILE *f = fopen(fname, "r");
    if (!f)
        goto invalid;
    cfg->type_count = 0;

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        char *s = trim(line);
        if (*s == '\0' || *s == '#')
            continue;

        char *eq = strchr(s, '=');
        if (!eq)
            continue;

        *eq = '\0';
        char *key = trim(s);
        char *val = trim(eq + 1);

        if (strcmp(key, "fish_per_type") == 0) {
            if (!parse_boids_line(val, cfg))
                goto invalid;
        }
    }

    fclose(f);
    return 1;
invalid:
    if (f)
        fclose(f);

    // default config
    cfg->type_count = 3;
    cfg->boids_per_type[0] = 1000;
    cfg->boids_per_type[1] = 1000;
    cfg->boids_per_type[2] = 1000;
    return 0;
}

#endif /* PARAMS_H */
