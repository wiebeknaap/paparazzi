#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

extern "C" {
    #include "std.h"
    #include "modules/obstacle_detector/obstacle_detector.h"
    #include "modules/computer_vision/cv.h"
    #include "state.h"
    #include "firmwares/rotorcraft/navigation.h"
    #include "generated/flight_plan.h"
    #include "autopilot.h"

    extern struct video_config_t front_camera;
}

// Fix for OpenCV/Paparazzi macro overlap
#ifdef Ptr
#undef Ptr
#endif

// --- INTERNAL GLOBALS ---
static cv::dnn::Net yolo_net;
static bool camera_initialized = false;
static int16_t obstacle_free_confidence = 0;

enum navigation_state_t { SAFE, OBSTACLE_FOUND_STATE, SEARCH_FOR_SAFE_HEADING };
static enum navigation_state_t nav_state = SAFE;

// --- EXTERNAL GLOBALS (Visible to C files) ---
extern "C" {
    int32_t obstacle_found = 0;
    float area_threshold = 0.025f;
    float current_area = 0.0f;
}

// --- VISION CALLBACK ---
struct image_t * detect_obstacle_yolo(struct image_t *img, uint8_t camera_id) {
    (void)camera_id;
    if (!img || !img->buf || yolo_net.empty()) return img;

    try {
        // 1. Wrap the raw data
        cv::Mat raw(img->h, img->w, CV_8UC2, img->buf);
        cv::Mat bgr, rotated, bright, resized;

        // 2. Convert YUV to BGR
        cv::cvtColor(raw, bgr, cv::COLOR_YUV2BGR_UYVY);

        // 3. Rotate 90 degrees Left (Counter-Clockwise)
        // This happens BEFORE resizing so we don't distort the pixels
        cv::rotate(bgr, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);

        // 4. Apply Brightness/Contrast boost (to match your training data)
        // alpha = 1.3 (Contrast), beta = 20 (Brightness)
        cv::convertScaleAbs(rotated, bright, 1.3, 20);

        // 5. Resize the PROCESSED (bright) image to 256x256
        // This was your bug: you were using 'bgr' here instead of 'bright'
        cv::resize(bright, resized, cv::Size(256, 256));

        // Now use 'resized' for your blob
        cv::Mat blob = cv::dnn::blobFromImage(resized, 1.0/255.0, cv::Size(256, 256), cv::Scalar(), true, false);
        yolo_net.setInput(blob);

        // STABLE FORWARD PASS
        std::vector<cv::String> outNames = yolo_net.getUnconnectedOutLayersNames();
        std::vector<cv::Mat> outs;
        yolo_net.forward(outs, outNames);
        fprintf(stderr, "[YOLO] yolo has ran\n");
        if (!outs.empty()) {
            cv::Mat output = outs[0];
            int num_proposals = (output.dims == 3) ? output.size[1] : output.rows;
            int dims = (output.dims == 3) ? output.size[2] : output.cols;
            float* data = (float*)output.data;

            float max_score = 0.0f;
            int best_idx = -1;
            int best_class = -1;

            for (int i = 0; i < num_proposals; i++) {
                float* row = data + (i * dims);
                float obj_conf = row[4];

                if (obj_conf > 0.40f) {
                    float class0 = row[5]; // Obstacle
                    float class1 = row[6]; // Gate
                    float s0 = obj_conf * class0;
                    float s1 = obj_conf * class1;

                    if (s0 > max_score && s0 > 0.5f) { max_score = s0; best_idx = i; best_class = 0; }
                    else if (s1 > max_score && s1 > 0.5f) { max_score = s1; best_idx = i; best_class = 1; }
                }
            }

            if (best_idx != -1) {
                obstacle_found = 1;
                float* best_row = data + (best_idx * dims);
                current_area = (best_row[2] / 256.0f) * (best_row[3] / 256.0f);
                cv::circle(resized, cv::Point((int)best_row[0], (int)best_row[1]), 10, cv::Scalar(0,0,255), 2);
            } else {
                obstacle_found = 0;
                current_area = 0.0f;
            }
        }
        cv::imshow("YOLOv3-Tiny Bebop", resized);
        cv::waitKey(1);
    } catch (const std::exception& e) {
        fprintf(stderr, "[YOLO ERROR] %s\n", e.what());
    }
    return img;
}

// --- INITIALIZATION ---
void obstacle_detector_init(void) {
    obstacle_found = 0;
    camera_initialized = false;

    try {
        // We load TWO files now
        cv::String modelConfig = "/home/wiebe/paparazzi/sw/airborne/modules/obstacle_detector/models/yolov3-tiny.cfg";
        cv::String modelWeights = "/home/wiebe/paparazzi/sw/airborne/modules/obstacle_detector/models/best.weights";

        // readNetFromDarknet is the "Old Reliable"
        yolo_net = cv::dnn::readNetFromDarknet(modelConfig, modelWeights);

        yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        fprintf(stderr, "[YOLO] DARKNET LOADED! No more ONNX crashes.\n");
    } catch (const std::exception& e) {
        fprintf(stderr, "[YOLO FATAL] Darknet Load Error: %s\n", e.what());
    }
}

// --- PERIODIC ---
void obstacle_detector_periodic(void) {
    if (!camera_initialized) {
        cv_add_to_device(&front_camera, (cv_function)detect_obstacle_yolo, 0, 0);
        camera_initialized = true;
        return;
    }

    if (!autopilot_in_flight()) return;

    if (obstacle_found == 0) obstacle_free_confidence = std::min(10, (int)obstacle_free_confidence + 1);
    else obstacle_free_confidence = 0;

    float psi = stateGetNedToBodyEulers_f()->psi;
    struct EnuCoor_i* pos = stateGetPositionEnu_i();

    switch (nav_state) {
        case SAFE:
            if (obstacle_found && current_area > area_threshold) {
                nav_state = OBSTACLE_FOUND_STATE;
            } else {
                waypoint_move_xy_i(WP_GOAL, pos->x + POS_BFP_OF_REAL(sinf(psi) * 1.5f),
                                            pos->y + POS_BFP_OF_REAL(cosf(psi) * 1.5f));
            }
            break;
        case OBSTACLE_FOUND_STATE:
            nav_state = SEARCH_FOR_SAFE_HEADING;
            break;
        case SEARCH_FOR_SAFE_HEADING:
            nav_set_heading_rad(psi + RadOfDeg(20.0f));
            if (obstacle_free_confidence >= 6) nav_state = SAFE;
            break;
    }
}