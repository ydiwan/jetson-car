/**
 * @file ld_preprocess.cpp
 * @brief This file contains the implementation file for class for preprocessing
 * the image of the road for the lane detection algrothrim in the lane_detector
 * class.
 *
 * @author Trevin Vaughan
 * @date 2025-07-29
 *
 * @par License:
 *      MIT License
 *
 * @par Description:
 *      - Does processing of image so that is can  be ready to be lane line
 * segment scanning. This intails:
 *          - cropping image
 *          - scale resolution
 *          - processing binary color mask
 *          - morphological filtering
 *
 * @par Requirements:
 *      - OpenCV >= 4.5.0
 *      - C++17 or higher
 *      - ROS2 Humble
 *
 * @par Changelog:
 *      - 2025-07-29: Initial creation
 */
#include "../include/lane_detection/ld_preprocessor.h"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

namespace preprocessor_ns {
// ============== Adding Names (Implementation Specific) =====================
// opencv names
using cv::Point;
using cv::resize;
using cv::Scalar;

// std names
using std::vector;

using namespace ld_config_ns;

// ============== Main Interface Function Implementations =====================
void Ld_preprocessor::run(Mat &rgb_img) {
    // crop and resize the rgb image
    crop_n_resize(rgb_img);

    // create a lane line color mask
    lane_mask(rgb_img);
}

// ============== Private Member Function Implementations =====================
void Ld_preprocessor::crop_n_resize(Mat &rgb_img) {
    // crop to the bottom part of the image
    int start_row = rgb_img.rows - (rgb_img.rows / config->scale_height);
    int end_row = rgb_img.rows;
    rgb_img = rgb_img.rowRange(start_row, end_row);

    // scale resolution
    resize(rgb_img, rgb_img, cv::Size(), config->scale_res, config->scale_res);
}

void Ld_preprocessor::lane_mask(const Mat &rgb_img) {
    // convert RGB image to HSV color space
    cv::cvtColor(rgb_img, this->hsv_img, cv::COLOR_BGR2HSV);

    // get White mask
    cv::inRange(this->hsv_img, cv::Scalar(0, 0, 190), cv::Scalar(180, 60, 255),
                this->white_mask);

    // get Yellow mask
    cv::inRange(this->hsv_img, cv::Scalar(20, 100, 100),
                cv::Scalar(30, 255, 255), this->yellow_mask);

    // combind the mask together
    cv::bitwise_or(this->white_mask, this->yellow_mask, this->bin_mask);

    // remove noise in mask using a morphological operation open
    morphologyEx(this->bin_mask, this->bin_mask, cv::MORPH_OPEN, kernel);

    // filter the bin_mask and updates the filter_bin_mask with the result
    filter_area();
}

void Ld_preprocessor::filter_area() {
    // find contours in the binary image
    vector<vector<Point>> contours;
    findContours(this->bin_mask, contours, cv::RETR_EXTERNAL,
                 cv::CHAIN_APPROX_SIMPLE);

    // update filter binary mask to the size of
    this->filter_bin_mask = Mat::zeros(this->bin_mask.size(), CV_8UC1);

    // go thought each contour
    for (const auto &contour : contours) {
        // get the Area of the contour blobs in the image
        double area = cv::contourArea(contour);

        if (area >= config->min_area && area <= config->max_area) {
            // Allow only shapes that are in with our area range
            cv::drawContours(filter_bin_mask,
                             std::vector<std::vector<Point>>{contour}, 0,
                             cv::Scalar(255), cv::FILLED);
        }
    }
}

} // namespace preprocessor_ns