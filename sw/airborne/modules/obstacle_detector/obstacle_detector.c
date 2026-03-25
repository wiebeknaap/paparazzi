#include "modules/obstacle_detector/obstacle_detector.h"
#include "modules/computer_vision/cv.h"
#include "state.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>

// --- GCS SETTINGS VARIABLES ---
// (These are initialized with your "known good" values)

// 1. Detection Mask (The Y-axis window)
int32_t scan_y_start = 80;
int32_t scan_y_end = 160;

// 2. Color Thresholds: Orange (Poles)
int32_t pole_u_min = 50;
int32_t pole_u_max = 130;
int32_t pole_v_min = 140;
int32_t pole_v_max = 250;

// 3. Color Thresholds: Green (Plants)
int32_t plant_u_min = 90;
int32_t plant_u_max = 120;
int32_t plant_v_min = 70;
int32_t plant_v_max = 115;

// 4. Sensitivity (Pixel Counts)
int32_t weight_pole = 400;
int32_t weight_plant = 600;

// 5. Navigation Settings
float turn_speed_avoid = 12.0f;    // Degrees to turn Left
float turn_speed_recover = 15.0f;  // Degrees to turn when OOB
float dist_forward_safe = 1.5f;    // Speed when clear
float dist_forward_avoid = 1.0f;   // Speed when avoiding

// --- INTERNAL GLOBALS ---
static int32_t orange_pixels = 0;
static int32_t green_pixels = 0;
static bool camera_initialized = false;
extern struct video_config_t front_camera;

enum nav_state_t { SAFE, AVOIDANCE, OUT_OF_BOUNDS };
static enum nav_state_t nav_state = SAFE;

// --- EXPERT HELPERS ---
bool is_pole(void) { return orange_pixels > weight_pole; }
bool is_plant(void) { return green_pixels > weight_plant; }

// --- WAYPOINT HELPER ---
static void move_waypoint_forward(uint8_t wp, float dist) {
    float cur_psi = stateGetNedToBodyEulers_f()->psi;
    float nx = stateGetPositionEnu_f()->x + sinf(cur_psi) * dist;
    float ny = stateGetPositionEnu_f()->y + cosf(cur_psi) * dist;
    waypoint_move_xy_i(wp, POS_BFP_OF_REAL(nx), POS_BFP_OF_REAL(ny));
}

// --- IMAGE CALLBACK (THE EYES) ---
struct image_t * detect_all(struct image_t *img, uint8_t camera_id) {
    (void)camera_id;
    orange_pixels = 0;
    green_pixels = 0;

    // Fixed the void pointer error: cast img->buf to uint8_t
    uint8_t* buf = (uint8_t*)img->buf;

    // Scan only the horizontal band defined by GCS
    for (int y = scan_y_start; y < scan_y_end; y++) {
        for (int x = 0; x < img->w; x++) {
            int idx = (y * img->w + x) * 2;
            uint8_t u = buf[idx];
            uint8_t v = buf[idx + 2];

            // Orange Detection
            if (u > pole_u_min && u < pole_u_max && v > pole_v_min && v < pole_v_max) {
                orange_pixels++;
            }
            // Green Detection
            else if (u > plant_u_min && u < plant_u_max && v > plant_v_min && v < plant_v_max) {
                green_pixels++;
            }
        }
    }
    return img;
}

// --- INITIALIZATION ---
void obstacle_detector_init(void) {
    nav_state = SAFE;
    camera_initialized = false;
}

// --- PERIODIC LOGIC (THE BRAIN) ---
void obstacle_detector_periodic(void) {
    // 1. Ensure camera is connected to the callback
    if (!camera_initialized) {
        cv_add_to_device(&front_camera, (cv_function)detect_all, 0, 0);
        camera_initialized = true;
        return;
    }

    if (!autopilot_in_flight()) return;

    // 2. Decide State based on Boundary and AI
    float drone_x = stateGetPositionEnu_f()->x;
    float drone_y = stateGetPositionEnu_f()->y;

    // IMPORTANT: Ensure 'ObstacleZone' matches your Flight Plan sector name
    if (!InsideObstacleZone(drone_x, drone_y)) {
        nav_state = OUT_OF_BOUNDS;
    }
    else if (is_pole() || is_plant()) {
        nav_state = AVOIDANCE;
    }
    else {
        nav_state = SAFE;
    }

    // 3. Execute State
    switch (nav_state) {
        case SAFE:
            move_waypoint_forward(WP_GOAL, dist_forward_safe);
            break;

        case AVOIDANCE:
            nav.heading += RadOfDeg(turn_speed_avoid);
            move_waypoint_forward(WP_GOAL, dist_forward_avoid);
            break;

        case OUT_OF_BOUNDS:
            nav.heading += RadOfDeg(turn_speed_recover);
            // Fly back to center waypoint (WP_TRAJECTORY)
            waypoint_move_xy_i(WP_GOAL, WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY));
            break;
    }
}