/**
 * @file ld_postprocessor.h
 * @brief This file contains header the class for post processing the result
from the lane detection in the lane_detector class.
 *
 * @author Trevin Vaughan <vaughantd@vcu.edu>
 * @date 2025-07-28
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
 *      - 2025-07-28: Initial implementation
 */
#ifndef LD_POSTPROCESSOR
#define LD_POSTPROCESSOR

#include <memory>
#include <opencv2/core.hpp>  //for cv::Mat
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>  // for rclcpp::Logger
#include <vector>

#include "ld_config.h"

/** @namespace anonymous namespace need for names from other classes */
namespace ld_postprocessor_ns {
// ============== Adding Names ===================================================
// opencv names
using cv::Mat;
using cv::Point;
using cv::Point2f;

// ROS2 names
using rclcpp::Logger;

// std names
using std::make_unique;
using std::vector;

// config namespace
using ld_config_ns::Ld_config;

/**
 * @class Ld_postprocessor
 * @brief Does processing for Lane_detector class
 *
 * This class does:
 *    * Create Visualization of the lane detection results
 *    * Calculates how many pixel is the center of the camera off from the
 * center of the road
 *    * Does a Bird's Eye View Transformation of the Road RGB image
 *
 *
 * @warning This is not designed to be use by itself, and should only use by the
 * lane_detector class.
 *
 * @warning Do not change the width of the image after creating this object. Create new
 * instance of this class if you need to change the width after construction.
 *
 * @example An example of how to use the class goes here.
 * @code{.cpp}
 *      @todo show how to use this class
 *      }
 * @endcode
 */
class Ld_postprocessor {
   private:
    // ============== Configuration=================================================
    std::unique_ptr<Ld_config> config;  // config settings

    // ============== Bird View Transformation Variables ============================
    vector<Point2f> bev_left_lane;   ///< contains the left lane pixel positions
    vector<Point2f> bev_right_lane;  ///< contains the right lane pixel positions
    Mat trans;  ///< Contain the Perspective Transform Matrix for BEV transformation

    int center_of_image;  ///< Center of the image in the X dimension
    // ============== Image Buffers ================================================
    Mat ld_result;  ///< RGB image of road with circles place where the lane
                    ///< lines where found
    Mat bev;        ///< Result of Bird's Eye View Transform of the road RGB image

    // ============== Private Member Functions =====================================
    /*******************************************************************************
     * @brief Get ROS2 logger for the this class
     * @return ROS2 logger object for this
     ******************************************************************************/
    static const Logger &get_logger() {
        static Logger logger = rclcpp::get_logger("Ld_postprocessor");
        return logger;
    }

    /*******************************************************************************
     * @brief Performs Bird's Eye View transformation on the road image and lane pixel
     * coordinates
     *
     * This function warps the RGB road image to a bird's eye view perspective using the
     * pre-calculated transformation matrix. It also transforms the left and right lane
     * pixel coordinates to match the BEV perspective for later use in symmetry
     * confidence calculations.
     *
     * @param rbg_img The input RGB image of the road (passed as const reference for
     * efficiency)
     * @param left_lane Vector of pixel coordinates representing the left lane line
     * @param right_lane Vector of pixel coordinates representing the right lane line
     *
     * @note Updates the following member variables:
     *       - bev: The bird's eye view transformed image
     *       - bev_left_lane: Transformed left lane coordinates in BEV perspective
     *       - bev_right_lane: Transformed right lane coordinates in BEV perspective
     ******************************************************************************/
    void bev_trans(Mat const &rbg_img, const vector<Point> &left_lane,
                   const vector<Point> &right_lane);

   public:
    // ============== Constructor(s)/Destructor(s) =================================
    /*******************************************************************************
     * @brief Construct a new Ld_postprocessor object. It creates the initalized the
     * Perspective transformation matrix (Ld_postprocessor::trans) for BEV transformation
     * calculation.
     *
     * @param config_ref a reference to the configuration setting struct.
     * @param img_width the width of the input image for the lane detection. This is need
     * for the Perspective transformation creation.
     ******************************************************************************/
    Ld_postprocessor(Ld_config const &config_ref, int img_width);

    // ============== Main Interface Function(s) ===================================
    /*******************************************************************************
     * @brief Main processing function for post-processing lane detection results
     *
     * This function performs the complete post-processing pipeline including:
     *  * Visualizing lane detection results by drawing colored circles on the image
     *    (blue for left lane, green for right lane, red for center)
     *  * Calculating the delta (horizontal pixel offset) between the center of the
     *    image and the detected center of the road
     *  * Performing Bird's Eye View transformation for confidence calculations
     *
     * @param rbg_img The input RGB image of the road
     * @param left_lane Vector of pixel coordinates for the detected left lane edge
     * @param right_lane Vector of pixel coordinates for the detected right lane edge
     * @param center_lane Vector of pixel coordinates for the calculated road center
     *
     * @return int The delta value (horizontal pixel offset) between image center and
     * road center. Positive values indicate road center is to the right of image center,
     *         negative values indicate it's to the left.
     *
     * @note The delta value is used by the vehicle controller to adjust steering
     ******************************************************************************/
    int run(Mat const &rbg_img, const vector<Point> &left_lane,
            const vector<Point> &right_lane, const vector<Point> &center_lane);

    // ============== Getters/Setter Function(s) ==============================
    vector<Point2f> get_bev_left_lane() const { return bev_left_lane; }
    vector<Point2f> get_bev_right_lane() const { return bev_right_lane; }

    Mat get_bev_img() const { return bev; }             ///< Returns a copy of BEV image
    const Mat &get_bev_img_ref() const { return bev; }  ///< reference to avoid copying

    /// Returns a copy of Lane Detection result image
    Mat get_ld_result() const { return ld_result; }
    /// reference to avoid copying
    const Mat &get_ld_result_ref() const { return ld_result; }
};
}  // namespace ld_postprocessor_ns
#endif
