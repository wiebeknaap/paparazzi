#include "simple_obstacle_avoider.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"

#include <math.h>
#include <stdbool.h>

#ifndef SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID
#define SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;

/* ================= PARAMETERS ================= */

float orange_threshold = 0.05f; //used to be 18

float forward_speed = 1.0f;
float yaw_rate_turn = 1.0f;

/* Timing */
static int stop_duration = 8;
static int cooldown_duration = 10;

/* ================= FSM ================= */

typedef enum {
  STATE_FORWARD,
  STATE_STOP,
  STATE_TURN,
  STATE_COOLDOWN
} avoid_state_t;

static avoid_state_t avoider_state = STATE_FORWARD;

/* Timers */
static int stop_timer = 0;
static int cooldown_timer = 0;

/* Turn direction */
static float turn_direction = 1.f;

/* Detection */
static float obstacle_strength = 0.f;
static int last_pixel_x = 160; // center of image (320/2)

/* ================= CALLBACK ================= */

static void color_detection_cb(uint8_t sender_id,
                               int16_t pixel_x,
                               int16_t pixel_y,
                               int16_t pixel_width,
                               int16_t pixel_height,
                               int32_t quality,
                               int16_t extra)
{
  (void)sender_id;
  (void)pixel_y;
  (void)pixel_width;
  (void)pixel_height;

  float frac = (float)quality / (320.f * 240.f);
  if (frac > 1.f) frac = 1.f;
  if (frac < 0.f) frac = 0.f;

  if (extra == 0) {
    obstacle_strength = frac;
    last_pixel_x = pixel_x; // store horizontal position
  }
}

/* ================= INIT ================= */

void simple_obstacle_avoider_init(void)
{
  obstacle_strength = 0.f;
  avoider_state = STATE_FORWARD;

  AbiBindMsgVISUAL_DETECTION(
    SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID,
    &color_detection_ev,
    color_detection_cb);
}

/* ================= MAIN ================= */

void simple_obstacle_avoider_periodic(void)
{
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    return;
  }

  switch (avoider_state) {

  /* ===== FORWARD ===== */
  case STATE_FORWARD:

    if (obstacle_strength > orange_threshold) {

      stop_timer = stop_duration;
      avoider_state = STATE_STOP;

      return;
    }

    guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
    guidance_h_set_body_vel(forward_speed, 0.f);
    break;

  /* ===== STOP ===== */
  case STATE_STOP:

    guidance_h_set_body_vel(0.f, 0.f);
    stop_timer--;

    if (stop_timer <= 0) {

      /* Turn AWAY from obstacle */
      if (last_pixel_x < 160)
        turn_direction = -1.f; // obstacle left → turn right
      else
        turn_direction = 1.f;  // obstacle right → turn left

      avoider_state = STATE_TURN;
    }
    break;

  /* ===== TURN UNTIL CLEAR ===== */
  case STATE_TURN:

    guidance_h_set_heading_rate(turn_direction * yaw_rate_turn);
    guidance_h_set_body_vel(0.f, 0.f);

    /* STOP TURNING when obstacle is gone */
    if (obstacle_strength < orange_threshold * 0.98f) { //used to be 0.5f

      cooldown_timer = cooldown_duration;
      avoider_state = STATE_COOLDOWN;

      obstacle_strength = 0.f; // reset
    }
    break;

  /* ===== COOLDOWN FORWARD ===== */
  case STATE_COOLDOWN:

    guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
    guidance_h_set_body_vel(forward_speed, 0.f);

    cooldown_timer--;

    if (cooldown_timer <= 0) {
      avoider_state = STATE_FORWARD;
    }
    break;
  }
}
