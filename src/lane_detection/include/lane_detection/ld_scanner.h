/**
 * @file ld_scanner.h
 * @brief This file contains the header file for class for scanning for lane
 * lines segments in a image.
 *
 * @author Trevin Vaughan
 * @date 2025-07-28
 *
 * @par License:
 *      MIT License
 *
 * @par Description:
 *      - This class takes an binary image and scans for lane line segments base
 * on it width and position in the image.

 * @par Requirements:
 *      - OpenCV >= 4.5.0
 *      - C++17 or higher
 *      - ROS2 Humble
 *
 * @par Changelog:
 *      - 2025-07-28: Initial implementation
 */
#ifndef LD_SCANNER
#define LD_SCANNER

#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>  // for rclcpp::Logger
#include <vector>

#include "ld_config.h"

/** @namespace ld_scanner_ns namespace need for names from other classes */
namespace ld_scanner_ns {
// ============== Adding Names ==============================================
// opencv names
using cv::Mat;
using cv::Point;

// ros2 names
using rclcpp::Logger;

// std names
using std::vector;

// config namespace
using ld_config_ns::Ld_config;

/******************************************************************************
 * @class Ld_scanner
 * @brief Scans binary images for lane line segments based on width and position
 *
 * @note That naming convention lane segments and lane lines are using
 * interchangeability. They identify by place a white pixels where the lane
 * is found.
 *
 * @warning This is not designed to be use by itself, and should only use by the
 * Lane_detector class.
 *
 * @example An example of how to use the class goes here.
 * @code{.cpp}
 *      Ld_config config(node);
 *      Ld_scanner scanner(config);
 *      Mat binary_mask = preprocessor.get_bin_mask();
 *      if (scanner.run(binary_mask)) {
 *          auto center_points = scanner.get_center_lane();
 *          // Process lane detection results
 *      }
 * @endcode
 ******************************************************************************/
class Ld_scanner {
   private:
    // ============== Configuration ===========================================
    Ld_config config;  ///< configuration setting struct from ld_config_ns::Ld_config

    // ============== Lane Data Storage =======================================
    vector<Point> left_lane;    ///< contains the left lane pixel positions
    vector<Point> right_lane;   ///< contains the right lane pixel positions
    vector<Point> center_lane;  ///< contains the center of lane pixel positions

    // ============== Private Member Functions ================================
    /// @brief Get ROS2 logger for the this class
    /// @return ROS2 logger object for this
    static const Logger &get_logger() {
        static Logger logger = rclcpp::get_logger("Ld_scanner");
        return logger;
    }

    /**************************************************************************
     * @brief Does a sliding window median filter the left, right, and
     * center lane scan results. It use the median_window and
     * median_threshold configuration setting for tweaking the median
     * filter.
     *
     * @note The member variables that are updated by this function:
     *    * Ld_scanner::left_lane
     *    * Ld_scanner::right_lane
     *    * Ld_scanner::center_lane
     **************************************************************************/
    void median_filter();

   public:
    // ============== Constructors/Destructors ================================
    /**
     * @brief Construct the lane scan. Start value for all vectors is empty.
     * @param config A reference to config settings
     */
    Ld_scanner(const Ld_config &config) : config(config) {}

    // ============== Main Interface Function(s) ==============================
    /**************************************************************************
     * @brief Scan for left, right lane. Store the pixel coordinates into
     * std:vector left_lane and right_lane. Calculates the center of road
     * pixel coordinates and store that in center_lane std::vector.
     *
     * @note The member variables that are updated by this function:
     *    * Ld_scanner::left_lane
     *    * Ld_scanner::right_lane
     *    * Ld_scanner::center_lane
     *
     * @param bin_mask A reference to binary mask for zero-copy.
     * @return true if any lanes were found.
     **************************************************************************/
    bool run(const Mat &bin_mask);

    // ============== Getters/Setter Function(s) ==============================
    vector<Point> get_left_lane() { return left_lane; }
    vector<Point> get_right_lane() { return right_lane; }
    vector<Point> get_center_lane() { return center_lane; }
};
}  // namespace ld_scanner_ns
#endif