/*
 * gate_guidance.c
 *
 * Gate guidance state machine — Paparazzi module implementation.
 *
 * ── How it fits into the airframe ────────────────────────────────────────────
 *
 *  gate_detector  (video thread)
 *       │  updates gate_latest  (mutex-protected, copied in periodic)
 *       ▼
 *  gate_guidance_periodic()  (autopilot thread, 15 Hz)
 *       │  reads gate_latest
 *       │  runs bg_update()  →  bg_command_t  (vx, vy, vz, yaw_rate)
 *       │  if gg_enabled && GUIDED mode:
 *       │      guidance_h_set_guided_body_vel(vx, vy)
 *       │      guidance_h_set_guided_heading_rate(yaw_rate)
 *       │      guidance_v_set_guided_vz(vz)
 *       ▼
 *  autopilot stabilization loop
 *
 * ── State machine (identical logic to the original bg_update) ─────────────────
 *
 *  SEARCH  ──(gate seen)──▶  ACQUIRE  ──(≥2 good frames)──▶  ALIGN
 *  ALIGN   ──(|ex|,|ey|<align_tol)──▶  APPROACH
 *  APPROACH──(|ex|,|ey|<commit_tol AND ow>18%)──▶  COMMIT
 *  COMMIT  ──(ow>35%)──▶  PASS
 *  PASS    ──(ow<10%)──▶  SEARCH          (gate passed through)
 *  any     ──(>lost_frame_limit misses)──▶  LOST ──▶  SEARCH
 *
 * ── Key differences from the standalone gate_guidance_c files ─────────────────
 *
 *  · bg_update() now takes unpacked scalar arguments instead of a pointer to
 *    the old gd_detection_t struct.  This removes the dependency on the old
 *    gate_detector_c.h header and makes the function directly unit-testable
 *    without any Paparazzi headers.
 *
 *  · gate_guidance_periodic() is the Paparazzi glue: it reads gate_latest,
 *    unpacks the fields, calls bg_update(), and forwards the resulting command
 *    to guidance_h / guidance_v when gg_enabled == 1 and the autopilot is in
 *    GUIDED mode.
 *
 *  · All config fields in gg_cfg are exported as GCS-settable dl_settings so
 *    you can tune gains live without recompiling.
 *
 * ── Vertical control note ─────────────────────────────────────────────────────
 *
 *  guidance_v_set_guided_vz() sets the vertical velocity in NED coordinates
 *  (+down).  The gate detector's vertical_error is (+up = gate above centre),
 *  so we negate: vz = +k_vert * ey  (+down when gate is above us).
 *
 *  If your Paparazzi build does not expose guidance_v_set_guided_vz(), set
 *  GATE_GUIDANCE_NO_VZ in your airframe and vertical will be held by the
 *  existing altitude hold instead.
 */

#include "gate_guidance.h"
#include "modules/gate_detector/gate_detector.h"

/* Paparazzi guidance APIs */
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"   /* autopilot_get_mode(), AP_MODE_GUIDED */

#include <math.h>    /* fabsf, fmaxf */
#include <string.h>  /* memset */
#include <stdio.h>   /* printf */

/* ── Internal helpers ────────────────────────────────────────────────────────*/

static float gg_clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ── Public globals ───────────────────────────────────────────────────────────*/

bg_config_t  gg_cfg;
bg_state_t   gg_state;
bg_command_t gg_cmd;
int          gg_enabled = 1;  /**< Set to 0 from GCS to disable command output */

/* ─────────────────────────────────────────────────────────────────────────────
 * bg_default_config — same defaults as the original standalone file.
 * ─────────────────────────────────────────────────────────────────────────────*/

bg_config_t bg_default_config(void)
{
  bg_config_t cfg;
  cfg.k_lat               = 0.90f;
  cfg.k_vert              = 0.80f;
  cfg.k_yaw               = 1.00f;
  cfg.search_yaw_rate     = 0.25f;   /* rad/s — slow clockwise search spin   */
  cfg.approach_vx         = 0.35f;   /* m/s                                  */
  cfg.commit_vx           = 0.65f;
  cfg.pass_vx             = 0.80f;
  cfg.align_tol           = 0.12f;   /* normalised image units [−1, +1]      */
  cfg.commit_tol          = 0.05f;
  cfg.confidence_acquire  = 0.45f;   /* min detector confidence to use       */
  cfg.lost_frame_limit    = 10;      /* frames before reverting to SEARCH    */
  return cfg;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * bg_reset — zero state, enter SEARCH.
 * ─────────────────────────────────────────────────────────────────────────────*/

void bg_reset(bg_state_t *st)
{
  memset(st, 0, sizeof(*st));
  st->mode = BG_SEARCH;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * bg_update — one step of the guidance state machine.
 *
 * Inputs
 * ------
 * valid    : detection is trustworthy this frame
 * conf     : detector confidence [0, 1]
 * ex       : lateral normalised error [−1, +1]  (gate_latest.lateral_error)
 * ey       : vertical normalised error [−1, +1] (gate_latest.vertical_error)
 * yaw_p    : yaw proxy from post-width asymmetry (gate_latest.yaw_proxy)
 * ow_frac  : opening_w / image_w  [0, 1]
 *
 * Outputs
 * -------
 * cmd      : body-frame velocity + yaw rate setpoints
 * ─────────────────────────────────────────────────────────────────────────────*/

void bg_update(const bg_config_t *cfg_in, bg_state_t *st,
               bool valid, float conf,
               float ex, float ey, float yaw_p, float ow_frac,
               bg_command_t *cmd)
{
  const bg_config_t cfg = cfg_in ? *cfg_in : bg_default_config();
  memset(cmd, 0, sizeof(*cmd));

  /* ── Frame counter update ──────────────────────────────────────────────── */

  const bool has_gate = valid && (conf >= cfg.confidence_acquire);

  if (has_gate) {
    st->good_frames++;
    st->lost_frames = 0;
  } else {
    st->lost_frames++;
    st->good_frames = 0;
  }

  /* ── Lost gate ─────────────────────────────────────────────────────────── */

  if (st->lost_frames > cfg.lost_frame_limit) {
    st->mode = BG_LOST;
  }

  /* ── State transitions (forward only) ─────────────────────────────────── */

  /* SEARCH → ACQUIRE when gate first appears */
  if (st->mode == BG_SEARCH && has_gate) {
    st->mode = BG_ACQUIRE;
  }

  if (has_gate) {
    const float ealign = fmaxf(fabsf(ex), fabsf(ey));

    /* ACQUIRE → ALIGN after 2 consecutive good detections */
    if (st->mode == BG_ACQUIRE && st->good_frames >= 2) {
      st->mode = BG_ALIGN;
    }

    /* ALIGN → APPROACH once errors are small */
    if (st->mode == BG_ALIGN && ealign < cfg.align_tol) {
      st->mode = BG_APPROACH;
    }

    /* APPROACH → COMMIT once well-aligned AND gate large enough */
    if (st->mode == BG_APPROACH
        && ealign < cfg.commit_tol
        && ow_frac > 0.18f) {
      st->mode = BG_COMMIT;
    }

    /* COMMIT → PASS once gate is very large (we are close) */
    if (st->mode == BG_COMMIT && ow_frac > 0.35f) {
      st->mode = BG_PASS;
    }
  }

  /* PASS → SEARCH once gate shrinks (passed through) */
  if (st->mode == BG_PASS && ow_frac < 0.10f) {
    st->mode = BG_SEARCH;
  }

  /* ── Commands when gate is NOT visible ────────────────────────────────── */

  if (!has_gate) {
    if (st->mode == BG_LOST || st->mode == BG_SEARCH) {
      /* Full search spin */
      st->mode        = BG_SEARCH;
      cmd->vx         = 0.0f;
      cmd->vy         = 0.0f;
      cmd->vz         = 0.0f;
      cmd->yaw_rate   = cfg.search_yaw_rate;
      cmd->mode       = st->mode;
      return;
    }
    /* Had gate briefly, don't spin hard — just a gentle correction */
    cmd->vx       = 0.0f;
    cmd->vy       = 0.0f;
    cmd->vz       = 0.0f;
    cmd->yaw_rate = 0.15f;
    cmd->mode     = st->mode;
    return;
  }

  /* ── Proportional controllers (gate is visible) ────────────────────────── */

  /* Lateral: positive ex = gate is to the right → fly right (vy > 0) */
  cmd->vy = gg_clampf(-cfg.k_lat  * ex, -0.6f, 0.6f);

  /* Vertical: positive ey = gate is below centre (NED: +down) → climb (vz < 0) */
  cmd->vz = gg_clampf(+cfg.k_vert * ey, -0.5f, 0.5f);

  /* Yaw: align with gate; also counter lateral drift with small ex term */
  cmd->yaw_rate = gg_clampf(-cfg.k_yaw * yaw_p - 0.40f * ex, -0.8f, 0.8f);

  /* ── State-specific forward velocity ──────────────────────────────────── */

  switch (st->mode) {
    case BG_SEARCH:
      cmd->vx       = 0.0f;
      cmd->yaw_rate = cfg.search_yaw_rate;
      break;

    case BG_ACQUIRE:
      /* Hover in place; wait for lock — do not advance yet */
      cmd->vx = 0.0f;
      break;

    case BG_ALIGN:
      /* Slow creep so we can see if alignment holds */
      cmd->vx = 0.05f;
      break;

    case BG_APPROACH:
      cmd->vx = cfg.approach_vx;
      break;

    case BG_COMMIT:
      /* Accelerate; reduce lateral/vertical corrections so we don't fight
       * the forward momentum at close range */
      cmd->vx  = cfg.commit_vx;
      cmd->vy *= 0.5f;
      cmd->vz *= 0.5f;
      break;

    case BG_PASS:
      /* Full-speed pass-through: only drive forward, no corrections */
      cmd->vx       = cfg.pass_vx;
      cmd->vy       = 0.0f;
      cmd->vz       = 0.0f;
      cmd->yaw_rate = 0.0f;
      break;

    case BG_LOST:
    default:
      cmd->vx       = 0.0f;
      cmd->vy       = 0.0f;
      cmd->vz       = 0.0f;
      cmd->yaw_rate = cfg.search_yaw_rate;
      st->mode      = BG_SEARCH;
      break;
  }

  cmd->mode = st->mode;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Mode name helper (for printf debugging)
 * ─────────────────────────────────────────────────────────────────────────────*/

static const char *bg_mode_name(bg_mode_t m)
{
  switch (m) {
    case BG_SEARCH:   return "SEARCH";
    case BG_ACQUIRE:  return "ACQUIRE";
    case BG_ALIGN:    return "ALIGN";
    case BG_APPROACH: return "APPROACH";
    case BG_COMMIT:   return "COMMIT";
    case BG_PASS:     return "PASS";
    case BG_LOST:     return "LOST";
    default:          return "?";
  }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * gate_guidance_init — Paparazzi module init (called once at boot)
 * ─────────────────────────────────────────────────────────────────────────────*/

void gate_guidance_init(void)
{
  gg_cfg = bg_default_config();
  bg_reset(&gg_state);
  memset(&gg_cmd, 0, sizeof(gg_cmd));
  gg_enabled = 1;

  printf("[gate_guidance] init: k_lat=%.2f k_vert=%.2f k_yaw=%.2f "
         "app_vx=%.2f cmt_vx=%.2f pass_vx=%.2f\n",
         (double)gg_cfg.k_lat, (double)gg_cfg.k_vert, (double)gg_cfg.k_yaw,
         (double)gg_cfg.approach_vx, (double)gg_cfg.commit_vx, (double)gg_cfg.pass_vx);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * gate_guidance_periodic — Paparazzi module periodic (15 Hz, autopilot thread)
 *
 * Reads gate_latest (already copied from video thread by gate_detector_periodic),
 * runs the state machine, then sends the resulting command to guidance_h / v
 * if gg_enabled && autopilot is in GUIDED mode.
 * ─────────────────────────────────────────────────────────────────────────────*/

void gate_guidance_periodic(void)
{
  /* ── Unpack gate_latest into plain scalars ─────────────────────────────── */

  const bool  valid   = gate_latest.valid;
  const float conf    = gate_latest.confidence;
  const float ex      = gate_latest.lateral_error;
  const float ey      = gate_latest.vertical_error;
  const float yaw_p   = gate_latest.yaw_proxy;

  /* Opening width as a fraction of image width [0, 1] */
  const float ow_frac = (gate_latest.image_w > 0u)
                        ? ((float)gate_latest.opening_w / (float)gate_latest.image_w)
                        : 0.0f;

  /* ── Run state machine ─────────────────────────────────────────────────── */

  bg_mode_t prev_mode = gg_state.mode;

  bg_update(&gg_cfg, &gg_state,
            valid, conf, ex, ey, yaw_p, ow_frac,
            &gg_cmd);

  /* Print on state change */
  if (gg_state.mode != prev_mode) {
    printf("[gate_guidance] %s -> %s  conf=%.2f  ex=%+.2f ey=%+.2f ow=%.2f\n",
           bg_mode_name(prev_mode), bg_mode_name(gg_state.mode),
           (double)conf, (double)ex, (double)ey, (double)ow_frac);
  }

  /* ── Send commands to autopilot ────────────────────────────────────────── */

  /*
   * Only send commands if:
   *   (a) gg_enabled is 1 (GCS toggle), AND
   *   (b) the autopilot is currently in GUIDED mode.
   *
   * This is important because the flight plan may switch the autopilot back
   * to NAV mode (e.g. for landing), and we must not fight that.
   */
  if (!gg_enabled) return;

#ifndef GATE_GUIDANCE_NO_AP_CHECK
  if (autopilot_get_mode() != AP_MODE_GUIDED) return;
#endif

  /*
   * guidance_h_set_guided_body_vel(vx, vy)
   *   vx  = body-forward velocity [m/s]
   *   vy  = body-right   velocity [m/s]
   *
   * guidance_h_set_guided_heading_rate(yaw_rate)
   *   [rad/s], positive = clockwise from above (right-hand NED)
   *
   * guidance_v_set_guided_vz(vz)
   *   [m/s], positive = downward (NED convention)
   *   cmd->vz is already in NED sign convention (see note in file header).
   *
   * If guidance_v_set_guided_vz is unavailable in your build, define
   * GATE_GUIDANCE_NO_VZ and vertical will be handled by altitude hold.
   */
  guidance_h_set_guided_body_vel(gg_cmd.vx, gg_cmd.vy);
  guidance_h_set_guided_heading_rate(gg_cmd.yaw_rate);

#ifndef GATE_GUIDANCE_NO_VZ
  guidance_v_set_guided_vz(gg_cmd.vz);
#endif
}
