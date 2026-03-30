#ifndef PLANT_AVOIDER_H
#define PLANT_AVOIDER_H
#include "std.h"

/* ── Grid dimensions (used in mav_exercise.c) ───────────── */
#define GRID_ROWS 40
#define GRID_COLS 32

/* ── Tunable parameters ─────────────────────────────────── */
#ifndef PLANT_AVOIDER_CAMERA
#define PLANT_AVOIDER_CAMERA front_camera
#endif
#ifndef PLANT_AVOIDER_FPS
#define PLANT_AVOIDER_FPS 4
#endif

#define SOBEL_THRESHOLD        20
#define EDGE_DENSITY_THRESHOLD 0.16f
#define MIN_VERTICAL_FILL      0.1f
#define TURN_THRESHOLD         0.6f

#define ROI_X_START 0.1f
#define ROI_X_END   0.9f
#define ROI_Y_START 0.1f
#define ROI_Y_END   0.9f

/* ── Result struct read by mav_exercise.c ───────────────── */
struct plant_avoider_result_t {
  bool     obstacle_detected;
  int32_t  safe_col;
  float    unsafe_ratio;
  bool     turn_left;
};

extern struct plant_avoider_result_t plant_avoider_result;

/* ── Module API ─────────────────────────────────────────── */
extern void plant_avoider_init(void);

/* Thread-safe snapshot getter — use this instead of reading the struct directly */
void plant_avoider_get_result(struct plant_avoider_result_t *out);

#endif /* PLANT_AVOIDER_H */
