/**
 * @file lane_detector.cpp
 * @brief This does lane detection using color mask and filter of shape base
 * (width, area, and ect).
 *
 *
 * @author Trevin Vaughan
 * @date 2025-07-28
 *
 * @par License:
 *      MIT License
 *
 * @par Description:
 *      This module implements computer vision-based lane detection using:
 *      - HSV color space segmentation for white/yellow lane marking
 *      - Bird's eye view transformation for accurate confidence measurements
 *      - Median filtering for outlier rejection
 *      - Dual confidence metrics (position and symmetry based)
 *
 * @section abbrev_list Abbreviation List
 *           - **ld*** - land detection
 *           - **img*** - image
 *           - **roi*** - Region of Interest
 *           - **ROS*** - Robot Operation System
 *           - **conf*** - confidence
 *           - **conf*** - confidence
 *           - **btm*** - bottom
 *           - **res*** - resolution
 *           - **bev*** - Bird's Eye View
 *
 * @par Requirements:
 *      - OpenCV >= 4.5.0
 *      - C++17 or higher
 *      - ROS2 Humble
 *
 * @par Changelog:
 *      - 2025-06-15: Initial creation
 */

#include "../include/lane_detection/lane_detector.h"

#include <kdl/frames.hpp>
#include <rclcpp/logging.hpp>
#include <type_traits>

namespace ld_ns {

// ============== Constructor/Destructor Implementations ===============================
Lane_detector::Lane_detector(rclcpp::Node *node)
    : config(node),
      preprocessor(config),
      scanner(config),
      postprocessor(config, config.rgb_img_width * config.scale_res),
      conf_calculator(config, (config.rgb_img_width * config.scale_res) / 2) {
    RCLCPP_DEBUG(get_logger(), "The width of the image: %f",
                 config.rgb_img_width * config.scale_res);

    // initialize ld_results with default values
    results.delta = 0;
    results.position_conf = 0.0;
    results.symmetry_conf = 0.0;
};

Lane_detector::Ld_results Lane_detector::step(Mat &rgb_img) {
    // process the rbg image
    preprocessor.run(rgb_img);

    // save results from preprocessor
    results.bin_mask = preprocessor.get_bin_mask();
    results.filter_mask = preprocessor.get_filter_bin_mask();

    // scan for road lane lines
    if (scanner.run(preprocessor.get_filter_bin_mask())) {
        // clang-format off
        results.delta = postprocessor.run(
            rgb_img, 
            scanner.get_left_lane(), 
            scanner.get_right_lane(),
            scanner.get_center_lane());
        // clang-format on

        results.ld_img = postprocessor.get_ld_result_ref();
        results.bev = postprocessor.get_bev_img_ref();

        results.position_conf = conf_calculator.cal_position_conf(results.delta);
        results.symmetry_conf = conf_calculator.cal_symmetrical_conf(
            postprocessor.get_bev_left_lane(), postprocessor.get_bev_right_lane());
    } else {
        // the lane_scan didn't find anything
        results.position_conf = 0.0;
        results.symmetry_conf = 0.0;
        results.delta = 0;
    }

    return results;
}
}  // namespace ld_ns