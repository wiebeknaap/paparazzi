#include "simple_obstacle_avoider.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"

#include <math.h>
#include <stdbool.h>

#ifndef VERBOSE
#define VERBOSE 1
#endif

#if VERBOSE
#include <stdio.h>
#define PRINT(string, ...) fprintf(stderr, "[simple_obstacle_avoider] " string, ##__VA_ARGS__)
#else
#define PRINT(string, ...)
#endif

#ifndef SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID
#define SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;
static float green_side_bias = 0.f;
static bool orange_updated = false;
static bool green_updated = false;

float orange_detect_threshold = 0.05f;
float floor_detect_threshold = 0.30f;
float plant_detect_threshold = 0.05f;

float middle_strong_threshold = 0.1f;
int low_conf_threshold = 1;
int high_conf_threshold = 3;
int max_confidence = 5;

float base_forward_speed = 0.25f;
float slow_forward_speed = 0.18f;
float very_slow_forward_speed = 0.12f;
float small_yaw_rate = 0.12f;
float medium_yaw_rate = 0.18f;
float search_yaw_rate = 0.10f;

static const float left_region_fraction   = 0.35f;
static const float middle_region_fraction = 0.30f;
static const float right_region_fraction  = 0.35f;

static const float raw_decay = 0.85f;

struct orange_info orange_raw;
struct green_info green_raw;
struct orange_info orange_filtered;
struct green_info green_filtered;
struct command last_command;
enum action last_action = SEARCH;
int obstacle_confidence = 0;

static float absvalue(float x){
  if (x < 0.f) {
    return -x;
  }
  return x;
}

static float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}


void update_detection_flags(struct orange_info *orange, struct green_info *green){
  if (orange != 0) {
    orange->left_detected = (orange->left_fraction >= orange_detect_threshold);
    orange->middle_detected = (orange->middle_fraction >= orange_detect_threshold);
    orange->right_detected = (orange->right_fraction >= orange_detect_threshold);
  }

  if (green != 0) {
    green->floor_visible = (green->floor_fraction >= floor_detect_threshold);
    green->plant_visible = (green->plant_fraction >= plant_detect_threshold);
  }
}

void set_orange_fractions(float left_fraction, float middle_fraction, float right_fraction){
  orange_raw.left_fraction = left_fraction;
  orange_raw.middle_fraction = middle_fraction;
  orange_raw.right_fraction = right_fraction;

  update_detection_flags(&orange_raw, 0);
}

void set_green_fractions(float floor_fraction, float plant_fraction){
  green_raw.floor_fraction = floor_fraction;
  green_raw.plant_fraction = plant_fraction;

  update_detection_flags(0, &green_raw);
}

int update_confidence(const struct orange_info *orange, const struct green_info *green){
  bool obstacle = false;

  if (orange != 0) {
    if (orange->left_detected || orange->middle_detected || orange->right_detected) {
      obstacle = true;
    }
  }

  if (green != 0) {
    if (green->plant_visible) {
      obstacle = true;
    }
  }

  if (obstacle) {
    obstacle_confidence += 2;
    if (obstacle_confidence > max_confidence) {
      obstacle_confidence = max_confidence;
    }
  }
  else {
    obstacle_confidence -= 1;
    if (obstacle_confidence < 0) {
      obstacle_confidence = 0;
    }
  }

  return obstacle_confidence;
}

enum action decide_action(const struct orange_info *orange, const struct green_info *green, int confidence){
  float left = 0.f;
  float middle = 0.f;
  float right = 0.f;
  bool plant_visible = false;

  if (orange != 0) {
    left = orange->left_fraction;
    middle = orange->middle_fraction;
    right = orange->right_fraction;
  }

  if (green != 0) {
    plant_visible = green->plant_visible;
  }

  if (confidence <= low_conf_threshold) {
      return FORWARD;
  }

  if (middle >= middle_strong_threshold) {
    if (left <= right) {
      return LEFT;
    }
    else {
      return RIGHT;
    }
  }

  if (left > middle && left > right) {
    return RIGHT;
  }

  if (right > middle && right > left) {
    return LEFT;
  }

  if (plant_visible && confidence >= high_conf_threshold) {
    if (green_side_bias < 0.f) {
      return RIGHT;   /* tree is on the left */
    } else {
      return LEFT;    /* tree is on the right */
    }
  }

  if (confidence >= high_conf_threshold) {
    if (left <= right) {
      return LEFT;
    }
    return RIGHT;
  }

  return FORWARD;
}

struct command action_to_command(enum action action, const struct orange_info *orange, const struct green_info *green){
  struct command command;
  float left = 0.f;
  float middle = 0.f;
  float right = 0.f;
  float side_error;
  float abs_error;
  float scaled_yaw;
  float yaw;
  bool floor_visible = false;

  command.forward_speed = 0.f;
  command.yaw_rate = 0.f;

  if (orange != 0) {
    left = orange->left_fraction;
    middle = orange->middle_fraction;
    right = orange->right_fraction;
  }

  if (green != 0) {
    floor_visible = green->floor_visible;
  }

  side_error = right - left;
  abs_error = absvalue(side_error);

  if (abs_error < 0.03f && !floor_visible){
    scaled_yaw = small_yaw_rate;
  } 
  else {
    scaled_yaw = medium_yaw_rate;
  }

  switch (action) {
    case FORWARD:
      yaw = 0.f;
      command.forward_speed = base_forward_speed;
      command.yaw_rate = yaw;
      break;

    case LEFT:
      if (middle >= middle_strong_threshold) {
        command.forward_speed = 0.f;
      } 
      else {
        command.forward_speed = very_slow_forward_speed;
      }
      command.yaw_rate = scaled_yaw;
      break;

    case RIGHT:
      if (middle >= middle_strong_threshold) {
        command.forward_speed = 0.f;
      } 
      else {
        command.forward_speed = very_slow_forward_speed;
      }
      command.yaw_rate = -scaled_yaw;
      break;

    case SEARCH:
      command.forward_speed = 0.f;
      command.yaw_rate = search_yaw_rate;
      break;

    case STOP:
      command.forward_speed = 0.f;
      command.yaw_rate = 0.f;
      break;
  }

  return command;
}

const char *action_name(enum action action){
  switch (action) {
    case FORWARD:
      return "FORWARD";
    case LEFT:
      return "LEFT";
    case RIGHT:
      return "RIGHT";
    case SEARCH:
      return "SEARCH";
    case STOP:
      return "STOP";
    default:
      return "UNKNOWN";
  }
}

static void update_orange_from_detection(int16_t pixel_x, int32_t quality)
{
  float img_w = (float)front_camera.output_size.w;
  float img_h = (float)front_camera.output_size.h;
  float img_area = img_w * img_h;

  if (img_w <= 1.f || img_h <= 1.f || img_area <= 1.f) {
    return;
  }

  float x_centered = (float)pixel_x;
  float x_img = x_centered + 0.5f * img_w;
  float x_norm = clampf(x_img / img_w, 0.f, 1.f);

  float frac = clampf((float)quality / img_area, 0.f, 1.f);

  float left = 0.f;
  float middle = 0.f;
  float right = 0.f;

  float left_end = left_region_fraction;
  float middle_end = left_region_fraction + middle_region_fraction;

  if (x_norm < left_end) {
    float t = x_norm / left_end;      
    left = frac * (1.f - 0.35f * t);
    middle = frac * (0.35f * t);
  } else if (x_norm < middle_end) {
    float t = (x_norm - left_end) / middle_region_fraction;  
    if (t < 0.5f) {
      float s = t / 0.5f;
      left   = frac * 0.30f * (1.f - s);
      middle = frac * (0.70f + 0.30f * (1.f - s));
    } else {
      float s = (t - 0.5f) / 0.5f;
      middle = frac * (1.0f - 0.30f * s);
      right  = frac * 0.30f * s;
    }
  } else {
    float t = (x_norm - middle_end) / right_region_fraction; 
    middle = frac * (0.35f * (1.f - t));
    right = frac * (1.f - 0.35f * (1.f - t));
  }

  set_orange_fractions(left, middle, right);
  orange_updated = true;
}



static void update_green_from_detection(int16_t pixel_x,int16_t pixel_y, int32_t quality)
{
  float img_w = (float)front_camera.output_size.w;
  float img_h = (float)front_camera.output_size.h;
  float img_area = img_w * img_h;

  if (img_w <= 1.f || img_h <= 1.f || img_area <= 1.f) {
    return;
  }

  float x_img = (float)pixel_x + 0.5f * img_w;
  float x_norm = clampf(x_img / img_w, 0.f, 1.f);

  float y_centered = (float)pixel_y;
  float y_img = 0.5f * img_h - y_centered;
  float y_norm = clampf(y_img / img_h, 0.f, 1.f);

  float frac = clampf((float)quality / img_area, 0.f, 1.f);

  float plant = 0.f;
  float floor = 0.f;

  if (y_norm < 0.5f) {
    float t = y_norm / 0.5f;
    plant = frac * (1.f - 0.25f * t);
    floor = frac * (0.25f * t);
  } else {
    float t = (y_norm - 0.5f) / 0.5f;
    plant = frac * (0.25f * (1.f - t));
    floor = frac * (0.75f + 0.25f * t);
  }

  set_green_fractions(floor, plant);

  /* negative = obstacle on left, positive = obstacle on right */
  green_side_bias = x_norm - 0.5f;

  green_updated = true;
}

static void color_detection_cb(uint8_t sender_id,
                               int16_t pixel_x,
                               int16_t pixel_y,
                               int16_t pixel_width,
                               int16_t pixel_height,
                               int32_t quality,
                               int16_t extra)
{
  (void)sender_id;
  (void)pixel_width;
  (void)pixel_height;

  if (extra == 0) {
    update_orange_from_detection(pixel_x, quality);
  } else if (extra == 1) {
    update_green_from_detection(pixel_x, pixel_y, quality);
  }
}

static void decay_raw_measurements_if_needed(void)
{
  if (!orange_updated) {
    set_orange_fractions(orange_raw.left_fraction * raw_decay,
                         orange_raw.middle_fraction * raw_decay,
                         orange_raw.right_fraction * raw_decay);
  }

  if (!green_updated) {
    set_green_fractions(green_raw.floor_fraction * raw_decay,
                        green_raw.plant_fraction * raw_decay);
  }

  orange_updated = false;
  green_updated = false;
}

void simple_obstacle_avoider_init(void)
{
  set_orange_fractions(0.f, 0.f, 0.f);
  set_green_fractions(0.f, 0.f);

  obstacle_confidence = 0;
  last_action = SEARCH;
  last_command.forward_speed = 0.f;
  last_command.yaw_rate = 0.f;

  orange_updated = false;
  green_updated = false;

  AbiBindMsgVISUAL_DETECTION(SIMPLE_OBSTACLE_AVOIDER_VISUAL_DETECTION_ID,
                             &color_detection_ev,
                             color_detection_cb);
}

void simple_obstacle_avoider_periodic(void)
{
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    obstacle_confidence = 0;
    last_action = SEARCH;
    last_command.forward_speed = 0.f;
    last_command.yaw_rate = 0.f;
    return;
  }

  decay_raw_measurements_if_needed();
  update_confidence(&orange_raw, &green_raw);

  last_action = decide_action(&orange_raw, &green_raw, obstacle_confidence);
  last_command = action_to_command(last_action, &orange_raw, &green_raw);

  if (last_command.yaw_rate == 0.f) {
    guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
  } else {
    guidance_h_set_heading_rate(last_command.yaw_rate);
  }

  guidance_h_set_body_vel(last_command.forward_speed, 0.f);

  PRINT("action=%s conf=%d orange=(%.3f %.3f %.3f) green=(%.3f %.3f) cmd=(%.3f %.3f)\n",
        action_name(last_action),
        obstacle_confidence,
        orange_raw.left_fraction,
        orange_raw.middle_fraction,
        orange_raw.right_fraction,
        green_raw.floor_fraction,
        green_raw.plant_fraction,
        last_command.forward_speed,
        last_command.yaw_rate);
}
