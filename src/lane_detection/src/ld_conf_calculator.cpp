/**
 * @file ld_conf_calculator.cpp
 * @brief Implementation file for Ld_conf_calculator class
 *
 * @author Trevin Vaughan <vaughantd@vcu.edu>
 * @date 2025-07-29
 *
 * @par License:
 *      MIT License
 *
 * @par Description:
 *      - Implementation of confidence calculation algorithms for lane detection
 *      - Calculates positional confidence based on lane center alignment
 *      - Calculates symmetrical confidence based on left/right lane geometry
 *
 * @par Requirements:
 *      - OpenCV >= 4.5.0
 *      - C++17 or higher
 *      - ROS2 Humble
 *
 * @par Changelog:
 *      - 2025-07-29: Initial creation
 */

#include "../include/lane_detection/ld_conf_calculator.h"

// Include additional headers needed for implementation
#include <algorithm>  // for std::sort, etc.
#include <cmath>      // for math functions
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <vector>

namespace ld_conf_calculator_ns {
// ============== Adding Names (Implementation Specific) =====================
// Additional using declarations not in header
using std::abs;
using std::max;
using std::min;

// ============== Constructor/Destructor Implementations ======================
Ld_conf_calculator::Ld_conf_calculator(Ld_config const& config, int center_of_image)
    : config(config), pos_conf(0.0), center_of_image(center_of_image) {
    // initialization logic
    // additional setup if needed
}

// ============== Main Interface Function Implementations =====================
double Ld_conf_calculator::cal_position_conf(int target_x) {
    double per_diff;
    double confidence;

    // Calculate percent difference between target x and center of image
    per_diff = abs((this->center_of_image - target_x) /
                   (double)((this->center_of_image + target_x) / 2));

    if (per_diff < this->config.pos_conf_threshold) {
        confidence = 1.0;  // High confidence if within threshold
    } else {
        // confidence = e ^ (-2.772588 * percent difference).
        // confidence will drop off exponentially as percent difference
        // increase. scale to pass through points (0.25,0.5)
        confidence = exp(EXP_SCALE * per_diff);
    }

    RCLCPP_DEBUG(get_logger(), "Position Confidence: %f", confidence);
    this->pos_conf = confidence;  // Store confidence for later use
    return confidence;
}

double Ld_conf_calculator::cal_symmetrical_conf(vector<cv::Point2f> bev_left_lane,
                                                vector<cv::Point2f> bev_right_lane) {
    if (bev_left_lane.empty() || bev_right_lane.empty()) {
        // double check if they're empty
        return 0.0;
    }

    // We dont need the float point percision so we store it in a int Point type
    Point last_left_point = bev_left_lane[0];
    Point last_right_point = bev_right_lane[0];

    double confidence = 1;
    double lowest_conf = DBL_MAX;  // highest double value possible
    double median_conf = 0;        // Median Confidence
    double final_conf = 0;         // final confidence
    vector<double> history_conf;   // history of confidence for iteration

    for (size_t i = 1; i < bev_left_lane.size(); i++) {
        // calculate the confidence for pixel coordinate
        Point right_point = bev_right_lane[i];
        Point left_point = bev_left_lane[i];
        double left_angle = 0;      // left lane angle from the bottom of the screen
        double right_angle = 0;     // right lane angle from the bottom of the screen
        double per_diff_angle = 0;  // percent different of left and right lanes angle

        // slope = rise over run
        double left_dy = (double)((left_point.y - last_left_point.y));
        double left_dx = (double)(left_point.x - last_left_point.x);
        // subtract result from 180 because we want the angle left side of
        // the line
        left_angle = 180.0 - atan2(left_dy, left_dx) * (180.0 / M_PI);

        double right_dy = (double)((right_point.y - last_right_point.y));
        double right_dx = (double)(right_point.x - last_right_point.x);
        right_angle = atan2(right_dy, right_dx) * (180 / M_PI);

        // calculate percent diff
        per_diff_angle =
            abs((left_angle - right_angle) / (double)((left_angle + right_angle) / 2));

        // the confidence will have the curve of 1-x^4, where is x is the
        // percent differences
        confidence = 1.0 - pow(per_diff_angle, 4);
        confidence = max(0.0, confidence);  // no negative values

        // save the this coordinates' confidence score
        history_conf.push_back(confidence);

        // save the lower value
        lowest_conf = min(confidence, lowest_conf);

        // update last points
        last_left_point = left_point;
        last_right_point = right_point;
    }

    // calculate median of the confidences
    sort(history_conf.begin(), history_conf.end());

    size_t middle_index = history_conf.size() / 2;
    if (history_conf.size() % 2 != 0) {
        // if the size is odd then the middle element is the median
        median_conf = history_conf[middle_index];
    } else {
        // if even median is the average of the 2 middle elements
        median_conf = (history_conf[middle_index - 1] + history_conf[middle_index]) / 2;
    }

    // 70% of score is made of median confidence and 30% is lowest confidence value found
    final_conf = 0.7 * median_conf + 0.3 * lowest_conf;

    RCLCPP_DEBUG(get_logger(), "Symmetrical Confidence: %f", final_conf);
    this->sym_conf = final_conf;
    return final_conf;
}
// ============== Private Member Function Implementations =====================
// Add private helper functions here as needed

// ============== Getter/Setter Implementations (if not inline) ===============
// Only include if these were declared but not defined in the header
// Most getters/setters should be inline in the header

}  // namespace ld_conf_calculator_ns