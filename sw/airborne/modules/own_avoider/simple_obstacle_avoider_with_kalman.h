#ifndef SIMPLE_OBSTACLE_AVOIDER_H
#define SIMPLE_OBSTACLE_AVOIDER_H

#include <stdint.h>
#include <stdbool.h>

struct kalman_1d {
  float x;
  float p;
  float q;
  float r;
};

struct orange_info {
  float left_fraction;
  float middle_fraction;
  float right_fraction;
  bool left_detected;
  bool middle_detected;
  bool right_detected;
};

struct green_info {
  float floor_fraction;
  float plant_fraction;
  bool floor_visible;
  bool plant_visible;
};

enum action {
  FORWARD = 0,
  LEFT,
  RIGHT,
  SEARCH,
  STOP
};

struct command {
  float forward_speed;
  float yaw_rate;
};


extern float orange_detect_threshold;
extern float floor_detect_threshold;
extern float plant_detect_threshold;
extern float middle_strong_threshold;
extern int low_conf_threshold;
extern int high_conf_threshold;
extern int max_confidence;
extern float base_forward_speed;
extern float slow_forward_speed;
extern float very_slow_forward_speed;
extern float small_yaw_rate;
extern float medium_yaw_rate;
extern float search_yaw_rate;

extern struct orange_info orange_raw;
extern struct green_info green_raw;
extern struct orange_info orange_filtered;
extern struct green_info green_filtered;
extern struct command last_command;
extern enum action last_action;
extern int obstacle_confidence;

void simple_obstacle_avoider_init(void);
void simple_obstacle_avoider_periodic(void);

void set_orange_fractions(float left_fraction, float middle_fraction, float right_fraction);
void set_green_fractions(float floor_fraction, float plant_fraction);

void kalman_init(struct kalman_1d *kf, float process_variance, float measurement_variance, float initial_value);
float kalman_update(struct kalman_1d *kf, float measurement);

void update_detection_flags(struct orange_info *orange, struct green_info *green);
void temporal_filter(void);
int update_confidence(const struct orange_info *orange, const struct green_info *green);
enum action decide_action(const struct orange_info *orange, const struct green_info *green, int confidence);
struct command action_to_command(enum action action, const struct orange_info *orange, const struct green_info *green);
const char *action_name(enum action action);

#endif
