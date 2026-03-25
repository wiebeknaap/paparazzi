#include "simple_obstacle_avoider.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h> // for printf

#ifndef SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID
#define SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;

/* ================= PARAMETERS ================= */

float orange_threshold = 0.18f;

float forward_speed = 1.0f;
float max_yaw_rate_turn = 1.0f;   // maximum turn rate
float turn_gain = 0.005f;         // how strongly the VTOL turns based on pixel offset

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
static float weighted_pixel_x_sum = 0.f;
static float total_blob_area = 0.f;
static int last_pixel_x = 160; // center of image (320/2)
static int blob_count = 0;     // number of blobs in the current frame

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

    float blob_area = (float)quality;
    if (blob_area < 0.f) blob_area = 0.f;

    if (extra == 0) {
        // accumulate weighted sum for average X
        weighted_pixel_x_sum += pixel_x * blob_area;
        total_blob_area += blob_area;
        blob_count++; // count each blob
    }
}

/* ================= INIT ================= */

void simple_obstacle_avoider_init(void)
{
    obstacle_strength = 0.f;
    avoider_state = STATE_FORWARD;
    weighted_pixel_x_sum = 0.f;
    total_blob_area = 0.f;
    blob_count = 0;

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

    // Compute weighted average X
    if (total_blob_area > 0.f) {
        last_pixel_x = (int)(weighted_pixel_x_sum / total_blob_area);
        obstacle_strength = total_blob_area / (320.f * 240.f); // normalize
    } else {
        obstacle_strength = 0.f;
    }

    // Print the number of blobs detected in this frame
    printf("Detected blobs in this frame: %d\n", blob_count);

    // Reset accumulators for next frame
    weighted_pixel_x_sum = 0.f;
    total_blob_area = 0.f;
    blob_count = 0;

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
                // Compute horizontal offset from center
                float offset = last_pixel_x - 160; // + → obstacle right, - → obstacle left

                // Set turn direction based on offset
                turn_direction = (offset > 0.f) ? 1.f : -1.f;

                // Set yaw rate proportional to magnitude of offset
                float yaw_rate = fminf(max_yaw_rate_turn, fabsf(offset) * turn_gain);
                max_yaw_rate_turn = yaw_rate;

                avoider_state = STATE_TURN;
            }
            break;

        /* ===== TURN UNTIL CLEAR ===== */
        case STATE_TURN:

            guidance_h_set_heading_rate(turn_direction * max_yaw_rate_turn);
            guidance_h_set_body_vel(0.f, 0.f);

            // Stop turning when obstacle is gone
            if (obstacle_strength < orange_threshold * 0.98f) {
                cooldown_timer = cooldown_duration;
                avoider_state = STATE_COOLDOWN;

                obstacle_strength = 0.f;
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
