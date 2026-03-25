/*
 * gate_detector.h
 *
 * Blue post-pair gate detector for Paparazzi — header.
 * See gate_detector.xml for installation and usage instructions.
 *
 * Quick usage:
 *   #include "modules/gate_detector/gate_detector.h"
 *   if (gate_latest.valid) {
 *       float lat = gate_latest.lateral_error;   // [-1,+1], 0=centred
 *       float vert = gate_latest.vertical_error; // [-1,+1], 0=centred
 *       float yaw  = gate_latest.yaw_proxy;      // gate rotation estimate
 *   }
 */

#ifndef GATE_DETECTOR_H
#define GATE_DETECTOR_H

#ifdef __GATE_MOCK_BUILD__
/* std.h provided by mock */
#else
#include "std.h"
#endif
#include <stdint.h>
#include <stdbool.h>

/* ── Camera and frame rate ─────────────────────────────────────────────────── */

#ifndef GATE_DETECTOR_CAMERA
#define GATE_DETECTOR_CAMERA front_camera
#endif

#ifndef GATE_DETECTOR_FPS
#define GATE_DETECTOR_FPS 15
#endif

/* ── YUV422 blue-pixel thresholds ─────────────────────────────────────────────
 * Paparazzi images are UYVY (4 bytes per 2 pixels: U, Y0, V, Y1).
 * U is the blue–yellow chroma difference (neutral = 128, blue = 160–240).
 * V is the red–green chroma difference  (neutral = 128, red  = 180–240).
 *
 * TU Delft gate blue (RGB ≈ 0, 114, 184):  U≈175, V≈95
 * Orange/cream pillars:                    U≈80,  V≈200   → rejected by U < U_MIN
 * Green foliage  (H<95 in HSV):            U≈100, V≈100   → rejected by U < U_MIN
 */
#ifndef GATE_U_MIN
#define GATE_U_MIN  130   /**< Min U for blue pixel (raise to reject green/teal) */
#endif
#ifndef GATE_U_MAX
#define GATE_U_MAX  255
#endif
#ifndef GATE_V_MIN
#define GATE_V_MIN  50    /**< Min V for blue pixel */
#endif
#ifndef GATE_V_MAX
#define GATE_V_MAX  150   /**< Max V for blue pixel (raise to allow more purple) */
#endif
#ifndef GATE_Y_MIN
#define GATE_Y_MIN  20    /**< Reject very dark pixels */
#endif
#ifndef GATE_Y_MAX
#define GATE_Y_MAX  220   /**< Reject overexposed pixels */
#endif

/* ── Geometry filters ─────────────────────────────────────────────────────── */

#ifndef GATE_POST_MIN_ASPECT       /* h/w ≥ 1.5: kills square furniture blobs   */
#define GATE_POST_MIN_ASPECT 1.5f
#endif
#ifndef GATE_POST_MAX_ASPECT       /* h/w ≤ 14:  kills person's legs / thin rods */
#define GATE_POST_MAX_ASPECT 14.0f
#endif
#ifndef GATE_POST_MIN_HEIGHT_FRAC  /* post height ≥ 5 % of frame               */
#define GATE_POST_MIN_HEIGHT_FRAC 0.05f
#endif
#ifndef GATE_COL_DENSITY_MIN       /* ≥ 18 % of post span must contain blue     */
#define GATE_COL_DENSITY_MIN 0.18f /* Kills shelf/tarp edges (col_frac ≈ 0.05) */
#endif
#ifndef GATE_MIN_GAP_TO_POST_W     /* gap ≥ 1.5 × max_post_width               */
#define GATE_MIN_GAP_TO_POST_W 1.5f/* Kills single-pole-split artefacts        */
#endif
#ifndef GATE_MAX_OPENING_ASPECT    /* opening_w / opening_h ≤ 2.2              */
#define GATE_MAX_OPENING_ASPECT 2.2f/* Real gate ≈ 1.0; blue banners ≈ 2.5–14  */
#endif
#ifndef GATE_MAX_MASK_DENSITY      /* if > 28 % of frame is blue → skip        */
#define GATE_MAX_MASK_DENSITY 0.28f
#endif

/* ── False-positive veto floors ───────────────────────────────────────────── */

#ifndef GATE_PURITY_U_HARD_FLOOR   /* mean U in post < 120 → orange/cream veto */
#define GATE_PURITY_U_HARD_FLOOR 120
#endif
#ifndef GATE_ISOLATION_HARD_FLOOR  /* blue must not extend past posts (banners) */
#define GATE_ISOLATION_HARD_FLOOR 0.30f
#endif

/* ── Scoring ──────────────────────────────────────────────────────────────── */

#ifndef GATE_MIN_CONFIDENCE
#define GATE_MIN_CONFIDENCE 0.50f  /**< Minimum score to declare a valid detection */
#endif

/* ── ABI sender ID ────────────────────────────────────────────────────────── */

#ifndef GATE_DETECTOR_ID
#define GATE_DETECTOR_ID 3         /**< VISUAL_DETECTION sender ID for gate module */
#endif

/* ── Output struct ───────────────────────────────────────────────────────────
 * Updated from the video thread (via mutex) and copied to gate_latest in the
 * periodic function — safe to read from any module's periodic function.
 */
struct gate_detection_t {
  bool    valid;            /**< true when confidence ≥ GATE_MIN_CONFIDENCE         */
  float   confidence;       /**< Weighted score in [0, 1]                           */
  int16_t center_x;         /**< Opening centre X, image pixels from left edge      */
  int16_t center_y;         /**< Opening centre Y, image pixels from top edge       */
  int16_t opening_w;        /**< Opening width in pixels                            */
  int16_t opening_h;        /**< Opening height in pixels                           */
  float   lateral_error;    /**< (cx − w/2) / (w/2)  →  range [−1, +1]             */
  float   vertical_error;   /**< (cy − h/2) / (h/2)  →  range [−1, +1]             */
  float   yaw_proxy;        /**< Post-width asymmetry → gate yaw estimate           */
  uint16_t image_w;         /**< Width of the frame this was detected in            */
  uint16_t image_h;         /**< Height of the frame this was detected in           */
};

/* ── GCS-settable parameters (also accessible from other modules) ─────────── */

extern uint8_t gate_u_min;
extern uint8_t gate_v_max;
extern float   gate_min_confidence;
extern uint8_t gate_purity_u_floor;
extern float   gate_col_density_min;
extern uint8_t gate_debug_overlay;  /**< 1 = annotate image stream with bounding box */

/* ── Latest detection — safe to read from periodic functions ──────────────── */

extern struct gate_detection_t gate_latest;

/* ── Public API ───────────────────────────────────────────────────────────── */

void gate_detector_init(void);
void gate_detector_periodic(void);

/* Handler stubs called by dl_settings (GCS slider changes) */
void gate_detector_set_u_min(uint8_t val);
void gate_detector_set_v_max(uint8_t val);

#endif /* GATE_DETECTOR_H */
