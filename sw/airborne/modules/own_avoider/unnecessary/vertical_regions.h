#ifndef VERTICAL_REGIONS_H
#define VERTICAL_REGIONS_H

#include <stdint.h>

struct VerticalRegions {
    float left_fraction;
    float middle_fraction;
    float right_fraction;
};

void compute_vertical_regions(
    const uint8_t *mask,
    int width,
    int height,
    struct VerticalRegions *out
);

#endif