#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include "std.h"

// Forward declaration to prevent include path errors in the global build
struct image_t;

// --- GCS SETTINGS (EXTERN) ---
// These allow the Settings XML to link to the variables in your .c file

// 1. Scan Window
extern int32_t scan_y_start;
extern int32_t scan_y_end;

// 2. Orange Thresholds
extern int32_t pole_u_min;
extern int32_t pole_u_max;
extern int32_t pole_v_min;
extern int32_t pole_v_max;

// 3. Green Thresholds
extern int32_t plant_u_min;
extern int32_t plant_u_max;
extern int32_t plant_v_min;
extern int32_t plant_v_max;

// 4. Sensitivity
extern int32_t weight_pole;
extern int32_t weight_plant;

// 5. Navigation
extern float turn_speed_avoid;
extern float turn_speed_recover;
extern float dist_forward_safe;
extern float dist_forward_avoid;

// --- MAIN FUNCTIONS ---
extern void obstacle_detector_init(void);
extern void obstacle_detector_periodic(void);
extern struct image_t * detect_all(struct image_t *img, uint8_t camera_id);

// --- EXPERT HELPERS ---
extern bool is_pole(void);
extern bool is_plant(void);

#endif /* OBSTACLE_DETECTOR_H */