#ifndef PARAMS_H
#define PARAMS_H

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

#endif /* PARAMS_H */
