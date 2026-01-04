#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>
#include <stdio.h>

typedef uint8_t u8;
typedef uint32_t u32;

typedef struct {
    float cohesion_r;
    float cohesion_strength;
    float separation_r;
    float separation_strength;
    float alignment_r;
    float alignment_strength;

    float min_speed;
    float max_speed;
} BoidsParams;

typedef struct {
    u8 type_count;
    u32 boids_per_type[256];
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

        cfg->boids_per_type[cfg->type_count++] = (u32)v;
        p = end;
    }

    return cfg->type_count > 0;
}

int config_parse(const char *fname, InitialConfig *cfg) {
    cfg->type_count = 1;
    cfg->boids_per_type[0] = 1000;

    FILE *f = fopen(fname, "r");
    if (!f)
        return 0;
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
            parse_boids_line(val, cfg);
        }
    }

    fclose(f);
    return 1;
}

#endif /* PARAMS_H */
