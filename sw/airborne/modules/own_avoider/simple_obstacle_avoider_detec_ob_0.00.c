#include "simple_obstacle_avoider.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID
#define SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;

/* ================= PARAMETERS ================= */

float orange_threshold = 0.005f;

float forward_speed = 0.6f;
float yaw_gain = 2.0f;

/* ================= FSM ================= */

typedef enum {
  STATE_FORWARD,
  STATE_AVOID
} avoid_state_t;

static avoid_state_t avoider_state = STATE_FORWARD;

/* Detection */
static float obstacle_strength = 0.f;
static int last_pixel_x = 160;

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
  (void)quality;

  /* ✅ ACCEPT ALL detections (NO FILTER) */
  if (pixel_width > 0 && pixel_height > 0) {

    float blob_size = (float)(pixel_width * pixel_height) / (320.f * 240.f);

    if (blob_size > 1.f) blob_size = 1.f;
    if (blob_size < 0.f) blob_size = 0.f;

    /* Keep biggest (nearest obstacle) */
    if (blob_size > obstacle_strength) {
      obstacle_strength = blob_size;
      last_pixel_x = pixel_x;
    }

    printf("DETECTED size=%f x=%d extra=%d\n",
           blob_size, pixel_x, extra);
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

  printf("Obstacle=%f\n", obstacle_strength);

  switch (avoider_state) {

  /* ===== FORWARD ===== */
  case STATE_FORWARD:

    if (obstacle_strength > orange_threshold) {
      printf(">>> AVOID\n");
      avoider_state = STATE_AVOID;
      return;
    }

    guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
    guidance_h_set_body_vel(forward_speed, 0.f);
    break;

  /* ===== AVOID ===== */
  case STATE_AVOID:

    /* Center-based steering */
    float error = ((float)last_pixel_x - 160.f) / 160.f;

    float yaw_rate = -yaw_gain * error;

    guidance_h_set_heading_rate(yaw_rate);
    guidance_h_set_body_vel(0.2f, 0.f);

    /* Back to forward if clear */
    if (obstacle_strength < orange_threshold * 0.5f) {
      printf(">>> CLEAR\n");
      avoider_state = STATE_FORWARD;
    }
    break;
  }

  /* Smooth decay */
  obstacle_strength *= 0.9f;
}
