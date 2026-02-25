/**
 * @file lane_detector.h
 * @brief This does lane detection using color mask and filter of shape base
 * (width, area, and ect).
 * Common bev
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
 *
 * @par Requirements:
 *      - OpenCV >= 4.5.0
 *      - C++17 or higher
 *      - ROS2 Humble
 *


 * @par Changelog:
 *      - 2025-06-15: Initial implementation
 *      - 2025-06-30: Confidence calculations
 *      - 2025-07-18: Added bird's eye view transformation
 *      - 2024-07-28: Reformated into separate classes rather than one bigger
 * helper class
 */
#ifndef LANE_DETECTOR
#define LANE_DETECTOR

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>  // for cv::Mat
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>  //for ros node

#include "ld_conf_calculator.h"
#include "ld_config.h"
#include "ld_postprocessor.h"
#include "ld_preprocessor.h"
#include "ld_scanner.h"

/** @namespace ld_ns namespace need for names from other classes */
namespace ld_ns {
// ============== Adding Names =========================================================
// opencv names
using cv::Mat;

// ROS2 names
using rclcpp::Logger;

// add member classs' namespaces
using ld_conf_calculator_ns::Ld_conf_calculator;
using ld_config_ns::Ld_config;
using ld_postprocessor_ns::Ld_postprocessor;
using ld_scanner_ns::Ld_scanner;
using preprocessor_ns::Ld_preprocessor;

/**
 * @class Lane_detector
 * @brief Main class for lane detection processing pipeline
 *
 * This class orchestrates the complete lane detection workflow including
 * preprocessing, detection, filtering, and confidence calculation.
 *
 *
 * @example
 * @code{.cpp}
 *      // somewhere in your ros node
 *      LaneDetector detector(config, road_img.size().width);
 *      auto result = detector.process(image);
 *      if (result.confidence > 0.8) {
 *          // Use detection results
 *      }
 * @endcode
 */
class Lane_detector {
   public:
    // ============== Constructors/Destructors =========================================
    /**
     * @brief
     *
     * @param node
     * @param rgb_img_width
     */
    Lane_detector(rclcpp::Node *node);

    // ============== Lane Detection Results Struct ====================================
    /**
     * @brief
     *
     */
    struct Ld_results {
        Mat ld_img;            ///< Lane position mark on the RGB image
        Mat bin_mask;          ///< binary mask of the lane lines
        Mat filter_mask;       ///< filtered binary mask of the lane lines
        Mat bev;               ///< Bird's Eye View image of the lane lines
        int delta;             ///< delta of the lane center from the center of the image
        double position_conf;  ///< confidence of the lane position
        double symmetry_conf;  ///< confidence of the lane symmetry
    };

    // ============== Main Interface Functions =========================================
    /**
     * @brief run frame pass in through the intire pipeline then the returns a struct
     * containing the results.
     *
     * @param rgb_img rbg frame to be process through the video pipeline
     * @return ld_results
     */
    Ld_results step(Mat &rgb_img);

   private:
    // =========== Sub Classes for lane Detection ======================================
    Ld_config config;                    ///< config setting struct
    Ld_preprocessor preprocessor;        ///< image processor
    Ld_scanner scanner;                  ///< Lane line scanner
    Ld_postprocessor postprocessor;      ///< postprocessor of results from the scanner
    Ld_conf_calculator conf_calculator;  ///< calculates our confidence in the results

    // ============== Private Member Functions =========================================
    /*******************************************************************************
     * @brief Get ROS2 logger for the this class
     * @return ROS2 logger object for this
     ******************************************************************************/
    static const Logger &get_logger() {
        static Logger logger = rclcpp::get_logger("lane_detector");
        return logger;
    }

    Ld_results results;  ///< results of the lane detection
};
}  // namespace ld_ns
#endif