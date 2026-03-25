#include "vertical_regions.h"

void compute_vertical_regions(
    const uint8_t *mask,
    int width,
    int height,
    struct VerticalRegions *out
) {
    int left_count = 0;
    int middle_count = 0;
    int right_count = 0;

    int total_pixels = width * height;

    // vertical region splits
    int left_end = 0.35f * width;
    int middle_end = 0.65f * width;

    for (int y = 0; y < height; y+=2) {
        for (int x = 0; x < width; x+=2) {

            int idx = y * width + x;

            if (mask[idx] == 0)
                continue;

            if (x < left_end) {
                left_count++;
            } 
            
            else if (x < middle_end) {
                middle_count++;
            } 
            
            else {
                right_count++;
            }
        }
    }

    if (total_pixels > 0) {
        out->left_fraction = (float)left_count / total_pixels;
        out->middle_fraction = (float)middle_count / total_pixels;
        out->right_fraction = (float)right_count / total_pixels;
    } 
    
    else {
        out->left_fraction = 0.0f;
        out->middle_fraction = 0.0f;
        out->right_fraction = 0.0f;
    }
}