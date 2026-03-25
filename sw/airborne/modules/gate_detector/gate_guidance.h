/*
 * gate_guidance.h
 *
 * Gate guidance state machine — Paparazzi module header.
 *
 * Reads gate_latest from gate_detector and issues body-frame velocity +
 * yaw-rate setpoints to the Paparazzi guidance_h / guidance_v controllers
 * while the autopilot is in GUIDED mode.
 *
 * Quick read from another module:
 *   #include "modules/gate_detector/gate_guidance.h"
 *   bg_mode_t mode = gg_state.mode;
 */

#ifndef GATE_GUIDANCE_H
#define GATE_GUIDANCE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── State machine modes ───────────────────────────────────────────────────── */

typedef enum {
  BG_SEARCH   = 0,   /**< No gate visible; spin in place searching            */
  BG_ACQUIRE  = 1,   /**< Gate appeared; hover and wait for a stable lock     */
  BG_ALIGN    = 2,   /**< Centering on gate opening; slow creep forward       */
  BG_APPROACH = 3,   /**< Gate centred; advance at approach_vx                */
  BG_COMMIT   = 4,   /**< Gate fills >18 % of frame; accelerate               */
  BG_PASS     = 5,   /**< Gate fills >35 % of frame; full-speed pass-through  */
  BG_LOST     = 6,   /**< Too many consecutive misses; revert to SEARCH       */
  BG_NUM_MODES
} bg_mode_t;

/* ── Output command struct ─────────────────────────────────────────────────── */

typedef struct {
  float     vx;        /**< Forward body velocity  [m/s]  (+forward)          */
  float     vy;        /**< Lateral body velocity  [m/s]  (+right)            */
  float     vz;        /**< Vertical body velocity [m/s]  (+down, NED)        */
  float     yaw_rate;  /**< Yaw rate [rad/s]  (+right = clockwise from above) */
  bg_mode_t mode;      /**< Active mode when this command was produced        */
} bg_command_t;

/* ── State struct (exposed so other modules can read the current mode) ──────── */

typedef struct {
  bg_mode_t mode;
  int       lost_frames;
  int       good_frames;
} bg_state_t;

/* ── Tuning config struct ──────────────────────────────────────────────────── */

typedef struct {
  /* Proportional controller gains */
  float k_lat;              /**< Lateral error → vy gain           (default 0.90) */
  float k_vert;             /**< Vertical error → vz gain          (default 0.80) */
  float k_yaw;              /**< Yaw proxy → yaw_rate gain         (default 1.00) */

  /* Search behaviour */
  float search_yaw_rate;    /**< Yaw rate while searching [rad/s]  (default 0.25) */

  /* Forward velocity setpoints per state [m/s] */
  float approach_vx;        /**< APPROACH state                    (default 0.35) */
  float commit_vx;          /**< COMMIT state                      (default 0.65) */
  float pass_vx;            /**< PASS state                        (default 0.80) */

  /* State-transition thresholds */
  float align_tol;          /**< max(|ex|,|ey|) < tol → APPROACH  (default 0.12) */
  float commit_tol;         /**< max(|ex|,|ey|) < tol → COMMIT    (default 0.05) */

  /* Detection quality gate */
  float confidence_acquire; /**< Min confidence to accept a detect (default 0.45) */
  int   lost_frame_limit;   /**< Misses before reverting to SEARCH (default 10)   */
} bg_config_t;

/* ── Public globals (GCS-settable via dl_settings in gate_guidance.xml) ─────── */

extern bg_config_t gg_cfg;      /**< Active config; all fields are GCS-settable  */
extern bg_state_t  gg_state;    /**< Current state machine state (read-only)     */
extern bg_command_t gg_cmd;     /**< Last computed command (read-only)           */
extern int         gg_enabled;  /**< 1 = send commands to AP, 0 = compute only  */

/* ── Core state-machine function (also callable standalone for unit-testing) ── */

/**
 * bg_default_config — fill a bg_config_t with the default gains.
 */
bg_config_t bg_default_config(void);

/**
 * bg_reset — zero the state struct and set mode = BG_SEARCH.
 */
void bg_reset(bg_state_t *st);

/**
 * bg_update — run one step of the gate guidance state machine.
 *
 * @param cfg   Pointer to config struct; if NULL, defaults are used.
 * @param st    Mutable state.  Must persist across calls (static/global).
 * @param valid Whether the current detection is trustworthy.
 * @param conf  Detection confidence [0,1].
 * @param ex    Lateral error [−1, +1] (from gate_latest.lateral_error).
 * @param ey    Vertical error [−1, +1] (from gate_latest.vertical_error).
 * @param yaw_p Yaw proxy (from gate_latest.yaw_proxy).
 * @param ow_frac Opening width as fraction of image width (opening_w/image_w).
 * @param cmd   Output command filled by this function.
 */
void bg_update(const bg_config_t *cfg, bg_state_t *st,
               bool valid, float conf,
               float ex, float ey, float yaw_p, float ow_frac,
               bg_command_t *cmd);

/* ── Paparazzi module entry points ──────────────────────────────────────────── */

void gate_guidance_init(void);
void gate_guidance_periodic(void);

#ifdef __cplusplus
}
#endif

#endif /* GATE_GUIDANCE_H */
