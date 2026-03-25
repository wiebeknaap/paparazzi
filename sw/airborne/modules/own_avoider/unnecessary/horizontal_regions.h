#ifndef HORIZONTAL_REGIONS_H
#define HORIZONTAL_REGIONS_H

#include <stdint.h>

struct HorizontalRegions {
    float floor_fraction;
    float plant_fraction;
};

void compute_horizontal_regions(
    const uint8_t *mask,
    int width,
    int height,
    struct HorizontalRegions *out
);

#endif