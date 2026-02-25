/**
 * @file ld_conf_calculator.h
 * @brief This file contain the ld_conf_calculator class. It calculates the
 * confidence of the lane detection results for the lane_detector class.
 *
 * @author Trevin Vaughan <vaughantd@vcu.edu>
 * @date 2025-07-29
 *
 * @par License:
 *      MIT License
 *
 * @par Description:
 *      - Calculate position confidence base on the position of where center of
 * lane is found.
 *      - Calculate symmetrical confidence base on how symmetrical are the left
 and right lanes are to each other.
 *
 * @par Requirements:
 *      - OpenCV >= 4.5.0
 *      - C++17 or higher
 *      - ROS2 Humble
 *
 * @par Changelog:
 *      - 2025-07-29: Initial implementation
 */
#ifndef CONF_CALCULATOR
#define CONF_CALCULATOR

#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>  // for rclcpp::Logger
#include <vector>             // for std::vector

#include "ld_config.h"

/** @namespace ld_conf_calculator namespace need for names from other classes */
namespace ld_conf_calculator_ns {
// ============== Adding Names =========================================================
// opencv names
using cv::Point;

// rclcpp names
using rclcpp::Logger;

// std names
using std::vector;

// lane detector lib namespaces
using ld_config_ns::Ld_config;

/***************************************************************************************
 * @class Ld_conf_calculator
 * @brief ld_conf_calculator class is used to calculate how confident should we be in the
 * lane detection results.
 *
 * There are two types of confidences scores this class calculates, positional confidence
 * and symmetrical confidence.

 * * Positional confidence is confidence score base on does the
 * position of the center make sense. If the algrothrim is working correctly, the center
 * lane of the lane is going to be aligned with the center of the camera, if is it not
 * that means our algrothrim is leading us off the road.
 *
 * * Symmetrical confidence is a confidence score base on if the left and right lane's
 * geometry is symmetrical. The left and right lanes will have the same slope from a top
 * down view of the road, hence why we do a Bird's Eye View transformation of perspective
 * so we are accurately calculate the left and right lane symmetry.
 *
 * Both confidence score are a value between 1 and 0, where is 1 is 100 percent confident
 in the output and a score of 0 is 0 percent.
 *
 *
 * @example An example of how to use the class goes here.
 * @code{.cpp}
 *      @todo create a example of using the class
 * @endcode
 **************************************************************************************/
class Ld_conf_calculator {
   private:
    // ============== Configuration ====================================================
    Ld_config config;  ///< configuration setting struct from ld_config_ns::Ld_config

    // ============== Positional Confidence Vars =======================================
    double pos_conf;      ///< positional confidence
    int center_of_image;  ///< center of the image, need for calculating position conf
    const double EXP_SCALE = -2.772588;  ///< scale for exponential confidence drop off

    // ============== Symmetrical Confidence Vars ======================================
    double sym_conf;  ///< symmetrical confidence

    // ============== Private Member Functions =========================================
    /*******************************************************************************
     * @brief Get ROS2 logger for the this class
     * @return ROS2 logger object for this
     ******************************************************************************/
    static const Logger& get_logger() {
        static Logger logger = rclcpp::get_logger("Ld_conf_calculator");
        return logger;
    }

   public:
    // ============== Constructors/Destructors =========================================
    /***********************************************************************************
     * @brief Construct a new Ld_conf_calculator
     * @param config A reference to the configuration setting
     * @param center_of_image the center image, this need for positional confidence score
     *      calculation.
     **********************************************************************************/
    Ld_conf_calculator(Ld_config const& config, int center_of_image);

    // ============== Main Interface Functions =========================================
    /***********************************************************************************
     * @brief Calculate positional confidence based on target lane center position
     *
     * This function calculates how confident we should be in the lane detection
     * results based on where the detected lane center is positioned relative to the
     * camera center. If the algorithm is working correctly, the center of the lane
     * should be aligned with the center of the camera. Deviation from center
     * indicates the algorithm may be leading us off the road.
     *
     * @param target_x The x-coordinate of the detected lane center in image pixels
     * @return double Positional confidence score between 0.0 and 1.0, where 1.0
     *         represents 100 percent confidence and 0.0 represents 0 percent confidence
     **********************************************************************************/
    double cal_position_conf(int target_x);

    /***********************************************************************************
     * @brief Calculate symmetrical confidence based on left and right lane geometry
     *
     * This function calculates how confident we should be in the lane detection
     * results based on the symmetry between the left and right lane geometries.
     * In a Bird's Eye View transformation, the left and right lanes should have
     * similar slopes and be symmetrical to each other. High asymmetry indicates
     * poor detection quality or road conditions that don't match our assumptions.
     *
     * @param bev_left_lane Vector of 2D points representing the left lane boundary
     *                      in Bird's Eye View pixel coordinates
     * @param bev_right_lane Vector of 2D points representing the right lane boundary
     *                       in Bird's Eye View pixel coordinates
     * @return double Symmetrical confidence score between 0.0 and 1.0, where 1.0
     *         represents perfect symmetry and 0.0 represents no symmetry
     **********************************************************************************/
    double cal_symmetrical_conf(vector<cv::Point2f> bev_left_lane,
                                vector<cv::Point2f> bev_right_lane);

    // ============== Getter/Setter Functions ==========================================
    /** @brief gets position confidence score */
    double get_pos_conf() const { return pos_conf; }
    /** @brief gets symmetrical confidence score */
    double get_sym_conf() const { return sym_conf; }
};
}  // namespace ld_conf_calculator_ns
#endif
