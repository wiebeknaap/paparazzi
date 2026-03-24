#include "only_orange.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef VERBOSE
#define VERBOSE 0
#endif

#if VERBOSE
#include <stdio.h>
#define PRINT(string, ...) fprintf(stderr, "[simple_obstacle_avoider] " string, ##__VA_ARGS__)
#else
#define PRINT(string, ...)
#endif

#ifndef ONLY_ORANGE_VISUAL_DETECTION_ID
#define ONLY_ORANGE_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;
static bool orange_updated = false;

float orange_detect_threshold = 0.02f;
float middle_strong_threshold = 0.1f;
int low_conf_threshold = 1;
int high_conf_threshold = 3;
int max_confidence = 5;

static const float left_region_fraction   = 0.35f;
static const float middle_region_fraction = 0.30f;
static const float right_region_fraction  = 0.35f;

static const float raw_decay = 0.85f;

static struct kalman_1d kf_orange_left;
static struct kalman_1d kf_orange_middle;
static struct kalman_1d kf_orange_right;

struct orange_info orange_raw;
struct orange_info orange_filtered;
extern enum action last_action = SEARCH;
int obstacle_confidence = 0;

static float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void kalman_init(struct kalman_1d *kf, float process_variance, float measurement_variance, float initial_value){
  if (kf == 0) {
    return;
  }

  kf->x = initial_value;
  kf->p = 1.0f;
  kf->q = process_variance;
  kf->r = measurement_variance;
}

float kalman_update(struct kalman_1d *kf, float measurement){
  float kalman_gain;

  if (kf == 0) {
    return measurement;
  }

  kf->p = kf->p + kf->q;
  kalman_gain = kf->p / (kf->p + kf->r);
  kf->x = kf->x + kalman_gain * (measurement - kf->x);
  kf->p = (1.0f - kalman_gain) * kf->p;

  return kf->x;
}

void update_detection_flags(struct orange_info *orange){
  if (orange != 0) {
    orange->left_detected = (orange->left_fraction >= orange_detect_threshold);
    orange->middle_detected = (orange->middle_fraction >= orange_detect_threshold);
    orange->right_detected = (orange->right_fraction >= orange_detect_threshold);
  }
  else{
    return;
  }
}

void set_orange_fractions(float left_fraction, float middle_fraction, float right_fraction){
  orange_raw.left_fraction = left_fraction;
  orange_raw.middle_fraction = middle_fraction;
  orange_raw.right_fraction = right_fraction;

  update_detection_flags(&orange_raw);
}

void temporal_filter(void){
  orange_filtered.left_fraction = kalman_update(&kf_orange_left, orange_raw.left_fraction);
  orange_filtered.middle_fraction = kalman_update(&kf_orange_middle, orange_raw.middle_fraction);
  orange_filtered.right_fraction = kalman_update(&kf_orange_right, orange_raw.right_fraction);

  update_detection_flags(&orange_filtered);
}

int update_confidence(const struct orange_info *orange){
  bool obstacle = false;

  if (orange != 0) {
    if (orange->left_detected || orange->middle_detected || orange->right_detected) {
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

enum action decide_action(const struct orange_info *orange, int confidence){
  float left = 0.f;
  float middle = 0.f;
  float right = 0.f;

  if (orange != 0) {
    left = orange->left_fraction;
    middle = orange->middle_fraction;
    right = orange->right_fraction;
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

  if (confidence >= high_conf_threshold) {
    if (left <= right) {
      return LEFT;
    }
    return RIGHT;
  }

  return FORWARD;
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

static void color_detection_cb(uint8_t sender_id,
                               int16_t pixel_x,
                               int16_t pixel_y,
                               int16_t pixel_width,
                               int16_t pixel_height,
                               int32_t quality,
                               int16_t extra){
  (void)sender_id;
  (void)pixel_width;
  (void)pixel_height;

  if (extra == 0) {
    update_orange_from_detection(pixel_x, quality);
  }
}

static void decay_raw_measurements_if_needed(void)
{
  if (!orange_updated) {
    set_orange_fractions(orange_raw.left_fraction * raw_decay,
                         orange_raw.middle_fraction * raw_decay,
                         orange_raw.right_fraction * raw_decay);
    }

  orange_updated = false;
}

void only_orange_init(void)
{
  set_orange_fractions(0.f, 0.f, 0.f);

  kalman_init(&kf_orange_left,   1e-4f, 5e-3f, 0.f);
  kalman_init(&kf_orange_middle, 1e-4f, 5e-3f, 0.f);
  kalman_init(&kf_orange_right,  1e-4f, 5e-3f, 0.f);

  obstacle_confidence = 0;
  last_action = SEARCH;
  orange_updated = false;

  AbiBindMsgVISUAL_DETECTION(ONLY_ORANGE_VISUAL_DETECTION_ID,
                             &color_detection_ev,
                             color_detection_cb);
}

void only_orange_periodic(void)
{

  decay_raw_measurements_if_needed();
  temporal_filter();
  update_confidence(&orange_filtered);

  last_action = decide_action(&orange_filtered, obstacle_confidence);
}
