/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "mav_exercise.h"

#include "plant_avoider.h"
#include "only_orange.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/gate_detector/gate_detector.h"
#include "modules/gate_detector/gate_guidance.h"
#include "state.h"
#include "autopilot.h"
#include "autopilot_static.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string, __FUNCTION__, ##__VA_ARGS__)

uint8_t increase_nav_heading(float incrementDegrees);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  OUT_OF_BOUNDS,
  HOLD
};

enum supervisor_mode_t {
  SUP_OBSTACLE = 0,
  SUP_GATE     = 1
};

/* existing obstacle globals */
float oa_color_count_frac = 1.0f;
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;
int16_t obstacle_free_confidence = 0;
float moveDistance = 0.5f;
float oob_haeding_increment = 3.f;
const int16_t max_trajectory_confidence = 2;
float divergence = 0.f;
static bool obstacle_entry = false;

/* gate-guidance globals declared in gate_guidance.h */
bg_config_t  gg_cfg;
bg_state_t   gg_state;
bg_command_t gg_cmd;
int          gg_enabled = 1;

static enum supervisor_mode_t sup_mode = SUP_OBSTACLE;

#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
#ifndef MAV_EXERCISE_OPTICAL_FLOW_ID
#define MAV_EXERCISE_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
#ifndef MAV_EXERCISE_HEADING_INCREMENT
#define MAV_EXERCISE_HEADING_INCREMENT 3.f
#endif
float oa_heading_increment = MAV_EXERCISE_HEADING_INCREMENT;

#ifndef MAV_EXERCISE_DIV_THRESHOLD
#define MAV_EXERCISE_DIV_THRESHOLD 0.3f
#endif
float divergence_threshold = MAV_EXERCISE_DIV_THRESHOLD;

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x,
                               int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width,
                               int16_t __attribute__((unused)) pixel_height,
                               int32_t quality,
                               int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t __attribute__((unused)) sender_id,
                            uint32_t __attribute__((unused)) stamp,
                            int32_t __attribute__((unused)) flow_x,
                            int32_t __attribute__((unused)) flow_y,
                            int32_t __attribute__((unused)) flow_der_x,
                            int32_t __attribute__((unused)) flow_der_y,
                            float __attribute__((unused)) quality,
                            float size_divergence)
{
  divergence = size_divergence;
}

/* -------------------------------------------------------------------------- */
/* Gate state-machine core                                                    */
/* -------------------------------------------------------------------------- */

static float gg_clampf(float v, float lo, float hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

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

bg_config_t bg_default_config(void)
{
  bg_config_t cfg;
  cfg.k_lat              = 0.90f;
  cfg.k_vert             = 0.80f;
  cfg.k_yaw              = 1.00f;
  cfg.search_yaw_rate    = 0.25f;
  cfg.approach_vx        = 0.35f;
  cfg.commit_vx          = 0.65f;
  cfg.pass_vx            = 0.80f;
  cfg.align_tol          = 0.12f;
  cfg.commit_tol         = 0.05f;
  cfg.confidence_acquire = 0.45f;
  cfg.lost_frame_limit   = 10;
  return cfg;
}

void bg_reset(bg_state_t *st)
{
  memset(st, 0, sizeof(*st));
  st->mode = BG_SEARCH;
}

void bg_update(const bg_config_t *cfg_in, bg_state_t *st,
               bool valid, float conf,
               float ex, float ey, float yaw_p, float ow_frac,
               bg_command_t *cmd)
{
  const bg_config_t cfg = cfg_in ? *cfg_in : bg_default_config();
  memset(cmd, 0, sizeof(*cmd));

  const bool has_gate = valid && (conf >= cfg.confidence_acquire);

  if (has_gate) {
    st->good_frames++;
    st->lost_frames = 0;
  } else {
    st->lost_frames++;
    st->good_frames = 0;
  }

  if (st->lost_frames > cfg.lost_frame_limit) {
    st->mode = BG_LOST;
  }

  if (st->mode == BG_SEARCH && has_gate) {
    st->mode = BG_ACQUIRE;
  }

  if (has_gate) {
    const float ax = fabsf(ex);
    const float ay = fabsf(ey);
    const float ealign = (ax > ay) ? ax : ay;

    if (st->mode == BG_ACQUIRE && st->good_frames >= 2) {
      st->mode = BG_ALIGN;
    }

    if (st->mode == BG_ALIGN && ealign < cfg.align_tol) {
      st->mode = BG_APPROACH;
    }

    if (st->mode == BG_APPROACH &&
        ealign < cfg.commit_tol &&
        ow_frac > 0.18f) {
      st->mode = BG_COMMIT;
    }

    if (st->mode == BG_COMMIT && ow_frac > 0.35f) {
      st->mode = BG_PASS;
    }
  }

  if (st->mode == BG_PASS && ow_frac < 0.10f) {
    st->mode = BG_SEARCH;
  }

  if (!has_gate) {
    if (st->mode == BG_LOST || st->mode == BG_SEARCH) {
      st->mode      = BG_SEARCH;
      cmd->vx       = 0.0f;
      cmd->vy       = 0.0f;
      cmd->vz       = 0.0f;
      cmd->yaw_rate = cfg.search_yaw_rate;
      cmd->mode     = st->mode;
      return;
    }

    cmd->vx       = 0.0f;
    cmd->vy       = 0.0f;
    cmd->vz       = 0.0f;
    cmd->yaw_rate = 0.15f;
    cmd->mode     = st->mode;
    return;
  }

  cmd->vy       = gg_clampf(-cfg.k_lat * ex, -0.6f, 0.6f);
  cmd->vz       = gg_clampf(+cfg.k_vert * ey, -0.5f, 0.5f);
  cmd->yaw_rate = gg_clampf(-cfg.k_yaw * yaw_p - 0.40f * ex, -0.8f, 0.8f);

  switch (st->mode) {
    case BG_SEARCH:
      cmd->vx       = 0.0f;
      cmd->yaw_rate = cfg.search_yaw_rate;
      break;

    case BG_ACQUIRE:
      cmd->vx = 0.0f;
      break;

    case BG_ALIGN:
      cmd->vx = 0.05f;
      break;

    case BG_APPROACH:
      cmd->vx = cfg.approach_vx;
      break;

    case BG_COMMIT:
      cmd->vx  = cfg.commit_vx;
      cmd->vy *= 0.5f;
      cmd->vz *= 0.5f;
      break;

    case BG_PASS:
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

/* -------------------------------------------------------------------------- */
/* Mode helpers                                                               */
/* -------------------------------------------------------------------------- */

static inline void set_nav_mode(void)
{
  autopilot_mode_auto2 = AP_MODE_NAV;
  autopilot_static_set_mode(AP_MODE_NAV);
}

static void update_gate_guidance(void)
{
  const bool  valid   = gate_latest.valid;
  const float conf    = gate_latest.confidence;
  const float ex      = gate_latest.lateral_error;
  const float ey      = gate_latest.vertical_error;
  const float yaw_p   = gate_latest.yaw_proxy;

  const float ow_frac = (gate_latest.image_w > 0u)
                      ? ((float)gate_latest.opening_w / (float)gate_latest.image_w)
                      : 0.0f;

  bg_mode_t prev_mode = gg_state.mode;

  bg_update(&gg_cfg, &gg_state, valid, conf, ex, ey, yaw_p, ow_frac, &gg_cmd);

  if (gg_state.mode != prev_mode) {
    PRINT("gate: %s -> %s | conf=%.2f ex=%+.2f ey=%+.2f ow=%.2f\n",
          bg_mode_name(prev_mode), bg_mode_name(gg_state.mode),
          (double)conf, (double)ex, (double)ey, (double)ow_frac);
  }
}

static bool gate_should_takeover(void)
{
  if (!gg_enabled) {
    return false;
  }

  if (!gate_latest.valid) {
    return false;
  }

  if (gate_latest.confidence < gg_cfg.confidence_acquire) {
    return false;
  }

  return (gg_state.mode == BG_APPROACH ||
          gg_state.mode == BG_COMMIT   ||
          gg_state.mode == BG_PASS);
}

static bool gate_should_release(void)
{
  if (!gg_enabled) {
    return true;
  }

  return (gg_state.mode == BG_SEARCH || gg_state.mode == BG_LOST);
}

static void apply_gate_command(void)
{
  /*
   * Branch-safe fallback:
   * use NAV-style heading + waypoint motion instead of GUIDED velocity
   * helper functions, which are not available on this branch.
   */

  float heading_step_deg = DegOfRad(gg_cmd.yaw_rate) * 0.20f;
  increase_nav_heading(heading_step_deg);

  /* push waypoints forward; trajectory a bit farther than goal */
  float traj_step = 0.10f + 0.20f * fmaxf(0.0f, gg_cmd.vx);
  float goal_step = 0.05f + 0.15f * fmaxf(0.0f, gg_cmd.vx);

  moveWaypointForward(WP_TRAJECTORY, traj_step);
  moveWaypointForward(WP_GOAL, goal_step);
}

static void on_enter_gate_mode(void)
{
  PRINT("SUPERVISOR: OBSTACLE -> GATE\n");

  waypoint_move_here_2d(WP_GOAL);
  waypoint_move_here_2d(WP_TRAJECTORY);

  /* stay in NAV on this branch-safe fallback */
  set_nav_mode();
  sup_mode = SUP_GATE;
}

static void on_exit_gate_mode(void)
{
  PRINT("SUPERVISOR: GATE -> OBSTACLE\n");

  set_nav_mode();

  waypoint_move_here_2d(WP_GOAL);
  waypoint_move_here_2d(WP_TRAJECTORY);

  navigation_state = SAFE;
  obstacle_entry = true;
  obstacle_free_confidence = max_trajectory_confidence;

  sup_mode = SUP_OBSTACLE;
}

/* -------------------------------------------------------------------------- */
/* Existing obstacle logic, kept intact but moved into helper                 */
/* -------------------------------------------------------------------------- */

static void run_obstacle_avoidance(void)
{
  struct plant_avoider_result_t pr;
  plant_avoider_get_result(&pr);

  bool orange_obstacle = (last_action == LEFT || last_action == RIGHT)
                         && (obstacle_confidence >= 2);

  PRINT("orange: %s (action=%s, conf=%d)\n",
        orange_obstacle ? "OBSTACLE" : "CLEAR",
        action_name(last_action), obstacle_confidence);

  bool fused_obstacle = orange_obstacle || pr.obstacle_detected;

  bool fused_turn_left;
  if (orange_obstacle) {
    fused_turn_left = (last_action == LEFT);
  } else {
    fused_turn_left = pr.turn_left;
  }

  PRINT("state: %d\n", navigation_state);
  PRINT("plant: unsafe=%.2f obs=%d turn_left=%d | orange: action=%s conf=%d\n",
        pr.unsafe_ratio, pr.obstacle_detected, pr.turn_left,
        action_name(last_action), obstacle_confidence);

  int32_t color_count_threshold = oa_color_count_frac
                                  * front_camera.output_size.w
                                  * front_camera.output_size.h;

  if (color_count < color_count_threshold) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;
  }
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  switch (navigation_state) {
    case SAFE:
      obstacle_entry = true;
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0
                 || divergence > divergence_threshold
                 || fused_obstacle) {
        navigation_state = OBSTACLE_FOUND;
      } else {
        float col_offset = (pr.safe_col - GRID_COLS / 2.f) / (GRID_COLS / 2.f);
        increase_nav_heading(col_offset * oa_heading_increment);
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      break;

    case OBSTACLE_FOUND:
      if (obstacle_entry) {
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_TRAJECTORY);
        obstacle_entry = false;
      }
      if (fused_obstacle) {
        float increment = fused_turn_left ? -oa_heading_increment : oa_heading_increment;
        increase_nav_heading(increment);
      } else {
        navigation_state = SAFE;
      }
      break;

    case OUT_OF_BOUNDS:
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      increase_nav_heading(-oob_haeding_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        increase_nav_heading(-oob_haeding_increment);
        navigation_state = SAFE;
      }
      break;

    case HOLD:
    default:
      break;
  }
}

/* -------------------------------------------------------------------------- */
/* Module entry points                                                        */
/* -------------------------------------------------------------------------- */

void mav_exercise_init(void)
{
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID,
                             &color_detection_ev, color_detection_cb);
  AbiBindMsgOPTICAL_FLOW(MAV_EXERCISE_OPTICAL_FLOW_ID,
                         &optical_flow_ev, optical_flow_cb);

  gg_cfg = bg_default_config();
  bg_reset(&gg_state);
  memset(&gg_cmd, 0, sizeof(gg_cmd));
  gg_enabled = 1;
  sup_mode = SUP_OBSTACLE;

  PRINT("init obstacle+gate supervisor | gate app_vx=%.2f commit_vx=%.2f pass_vx=%.2f\n",
        (double)gg_cfg.approach_vx,
        (double)gg_cfg.commit_vx,
        (double)gg_cfg.pass_vx);
}

void mav_exercise_periodic(void)
{
  if (!autopilot_in_flight()) {
    return;
  }

  /* always update gate state in background */
  update_gate_guidance();

  switch (sup_mode) {
    case SUP_OBSTACLE:
      if (autopilot_get_mode() == AP_MODE_GUIDED) {
        set_nav_mode();
      }

      run_obstacle_avoidance();

      if (gate_should_takeover()) {
        on_enter_gate_mode();
      }
      break;

    case SUP_GATE:
      /* branch-safe fallback: stay in NAV and move waypoints ourselves */
      if (autopilot_get_mode() == AP_MODE_GUIDED) {
        set_nav_mode();
      }

      apply_gate_command();

      if (gate_should_release()) {
        on_exit_gate_mode();
      }
      break;

    default:
      sup_mode = SUP_OBSTACLE;
      break;
  }
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading = stateGetNedToBodyEulers_f()->psi;
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * distanceMeters);
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * distanceMeters);
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}
