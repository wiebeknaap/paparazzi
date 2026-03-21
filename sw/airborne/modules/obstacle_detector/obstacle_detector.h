#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include "std.h"

#ifdef __cplusplus
extern "C" {
#endif

    // These variables must match the names in the .cpp exactly
    extern int32_t obstacle_found;
    extern float area_threshold;
    extern float current_area;

    // Module functions
    extern void obstacle_detector_init(void);
    extern void obstacle_detector_periodic(void);

#ifdef __cplusplus
}
#endif

#endif /* OBSTACLE_DETECTOR_H */