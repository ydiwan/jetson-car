/**
 * @file ld_postprocessor.cpp
 * @brief This file contains the class for post processing the result from the
 lane detection in the lane_detector class.
 *
 * @author Trevin Vaughan <vaughantd@vcu.edu>
 * @date 2025-08-12
 *
 * @par License:
 *      MIT License
 *
 * @par Description:
 *      - Create visualization of the lane detection results
 *      - Calculates the the delta x pixels that camera from the center of road
 *      - Does a Bird's Eye View Transformation which is use later on for the
//  symmetrical confidence calculation.
 *
 * @par Requirements:
 *      - OpenCV >= 4.5.0
 *      - C++17 or higher
 *      - ROS2 Humble
 *
 * @par Changelog:
 *      - 2025-08-12: Initial implementation
 */

#include "../include/lane_detection/ld_postprocessor.h"

#include <cstddef>
#include <memory_resource>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <vector>

namespace ld_postprocessor_ns {
// ============== Adding Names (Implementation Specific) ==============================
// opencv names
using cv::circle;

// std names
using std::ostringstream;
using std::setw;

// ============== Constructor/Destructor Implementations ===============================
Ld_postprocessor::Ld_postprocessor(Ld_config const &config_ref, int img_width)
    : config(make_unique<Ld_config>(config_ref)) {
    // Source pixel coordinates of road image to warp from
    vector<Point2f> src_points = {config->top_left_src, config->top_right_src,
                                  config->btm_left_src, config->btm_right_src};

    // destination points to warp the source points to
    Point2f top_left_dst(0.30 * img_width, config->top_left_src.y);
    Point2f bottom_left_dst(0.30 * img_width, config->btm_left_src.y);
    Point2f top_right_dst(0.70 * img_width, config->top_right_src.y);
    Point2f bottom_right_dst(0.70 * img_width, config->btm_left_src.y);

    vector<Point2f> dst_points = {top_left_dst, top_right_dst, bottom_left_dst,
                                  bottom_right_dst};

    // create transformation matrix
    this->trans = cv::getPerspectiveTransform(src_points, dst_points);

    // calculate center of the image
    this->center_of_image = img_width / 2;

    // shift the center of image left and right base on the configuration setting
    center_of_image = center_of_image + center_of_image * config->center_offset;
}

// ============== Main Interface Function Implementations ==============================
int Ld_postprocessor::run(Mat const &rgb_img, const vector<Point> &left_lane,
                          const vector<Point> &right_lane,
                          const vector<Point> &center_lane) {
    // shallow copy the rbg_img
    this->ld_result = rgb_img;

    // Add Visualization to Ld_postprocessor::ld_result
    RCLCPP_DEBUG(get_logger(), " Left Edge | Center | Right Edge ");
    RCLCPP_DEBUG(get_logger(), "----+----------+-----------+--------");
    for (size_t i = 0; i < center_lane.size(); i++) {
        // mark circles on the image
        circle(ld_result, left_lane[i], 1, cv::Scalar(255, 0, 0), -1);
        circle(ld_result, right_lane[i], 1, cv::Scalar(0, 255, 0), -1);
        circle(ld_result, center_lane[i], 1, cv::Scalar(0, 0, 255), -1);

        // create a OS stream for formating debugging output
        ostringstream oss;
        // clang-format off
        oss << setw(9)  << left_lane[i]    << " | " 
            << setw(10) << center_lane[i]  << " | " 
            << setw(6)  << right_lane[i]   << " | ";
        // clang-format on

        RCLCPP_DEBUG(get_logger(), "%s", oss.str().c_str());
    }

    // get the target pixel position we want the car to align itself with.
    // this will be the closest point to the front of the car.
    Point target = center_lane.back();
    RCLCPP_DEBUG(get_logger(), "target: (%d, %d)", target.x, target.y);

    // calculate delta. Delta is the distance in pixels the center of image is from the
    // center of road in the X dimension. So if the center of image is x = 100 and the
    // target is at  x = 105 and y = 50 pixel coordinate position,
    // then delta = 105 - 100 = 5 pixels.
    int delta = target.x - this->center_of_image;
    RCLCPP_DEBUG(get_logger(), "delta: %d", delta);

    // do BEV transformation on the lane detection result
    bev_trans(ld_result, left_lane, right_lane);

    return delta;
}

// ============== Private Member Function Implementations ==============================
void Ld_postprocessor::bev_trans(Mat const &rbg_img, const vector<Point> &left_lane,
                                 const vector<Point> &right_lane) {
    // warp the road image to Bird's Eye View
    cv::warpPerspective(rbg_img, this->bev, this->trans, rbg_img.size());

    // warp the left and right lane line pixel coordinate vector to match the BEV.
    vector<Point2f> left_float;
    vector<Point2f> right_float;
    for (size_t i = 0; i < left_lane.size(); i++) {
        // need to convert to floating point
        left_float.push_back(Point2f(left_lane[i]));
        right_float.push_back(Point2f(right_lane[i]));
    }

    cv::perspectiveTransform(left_float, bev_left_lane, trans);
    cv::perspectiveTransform(right_float, bev_right_lane, trans);
}

}  // namespace ld_postprocessor_ns
