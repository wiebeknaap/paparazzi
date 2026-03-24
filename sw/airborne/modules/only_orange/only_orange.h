#ifndef ONLY_ORANGE_H
#define ONLY_ORANGE_H

#include <stdint.h>
#include <stdbool.h>


struct orange_info {
  float left_fraction;
  float middle_fraction;
  float right_fraction;
  bool left_detected;
  bool middle_detected;
  bool right_detected;
};


struct kalman_1d {
  float x;
  float p;
  float q;
  float r;
};

enum action {
  FORWARD = 0,
  LEFT,
  RIGHT,
  SEARCH,
  STOP
};

extern float orange_detect_threshold;
extern float middle_strong_threshold;
extern int low_conf_threshold;
extern int high_conf_threshold;
extern int max_confidence;

extern struct orange_info orange_raw;
extern struct orange_info orange_filtered;
extern int obstacle_confidence;

void only_orange_init(void);
void only_orange_periodic(void);

void set_orange_fractions(float left_fraction, float middle_fraction, float right_fraction);
void update_detection_flags(struct orange_info *orange);
void temporal_filter(void);
int update_confidence(const struct orange_info *orange);
enum action decide_action(const struct orange_info *orange, int confidence);
const char *action_name(enum action action);

void kalman_init(struct kalman_1d *kf, float process_variance, float measurement_variance, float initial_value);
float kalman_update(struct kalman_1d *kf, float measurement);
#endif