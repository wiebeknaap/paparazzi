#include "horizontal_regions.h"

void compute_horizontal_regions(
    const uint8_t *mask,
    int width,
    int height,
    struct HorizontalRegions *out
) {
    int top_count = 0;
    int bottom_count = 0;

    int total_pixels = width * height;

    int split_y = (int)(0.5f * height);

    for (int y = 0; y < height; y+=2) {
        for (int x = 0; x < width; x+=2) {

            int idx = y * width + x;

            if (mask[idx] == 0)
                continue;

            if (y < split_y) {
                top_count++;     // plant
            } 
            
            else {
                bottom_count++;  // floor
            }
        }
    }

    if (total_pixels > 0) {
        out->plant_fraction = (float)top_count / total_pixels;
        out->floor_fraction = (float)bottom_count / total_pixels;
    } 
    
    else {
        out->plant_fraction = 0.0f;
        out->floor_fraction = 0.0f;
    }
}