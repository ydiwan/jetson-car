/**
 * @file ld_preprocess.h
 * @brief This file contains the header file for class for preprocessing the
 * image of the road for the lane detection algrothrim in the lane_detector
 * class.
 *
 * @author Trevin Vaughan
 * @date 2025-07-28
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
 *      - 2025-07-28: Initial implementation
 */
#ifndef LD_PREPROCESS
#define LD_PREPROCESS

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>  // for cv::Mat
#include <opencv2/imgproc.hpp>   // for cv::MORPH_ELLLIPSE
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>  // for rclcpp::Logger

#include "ld_config.h"

/** @namespace preprocessor_ns namespace need for names from other classes */
namespace preprocessor_ns {
// ============== Adding Names ==============================================
// opencv names
using cv::Mat;

// ros2 names
using rclcpp::Logger;

// std names
using std::make_unique;
using std::unique_ptr;

// config namespace
using ld_config_ns::Ld_config;

/******************************************************************************
 * @class Ld_preprocessor
 * @brief this class is use to for do image preprocessing for the Lane_detector
 * class.
 *
 * @warning This is not designed to be use by itself, and should only use by the
 * lane_detector class.
 *
 *
 *
 *
 ******************************************************************************/
class Ld_preprocessor {
   private:
    // ============== Configuration============================================
    std::unique_ptr<Ld_config> config;  // config settings

    // ============== Image Buffers ===========================================
    Mat bin_mask;         // binary image for color mask of lane lines
    Mat filter_bin_mask;  // filter binary color mask

    // for internal processing therefore they dont have getters
    Mat hsv_img;
    Mat white_mask;
    Mat yellow_mask;

    // ============== Constants ===============================================
    const Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));

    // ============== Private Member Functions ================================
    /**************************************************************************
     * @brief Get ROS2 logger for the this class
     * @return ROS2 logger object for this
     **************************************************************************/
    static const Logger &get_logger() {
        static Logger logger = rclcpp::get_logger("Ld_preprocessor");
        return logger;
    }

    /**************************************************************************
     * @brief crops and resize the image.
     * The height is divid by the scale_height config setting. So if
     *scale_height is 3, then you get the bottom 1/3 of the image. For
     *resolution, it's scale multiplied by the scale_res config setting. If the
     *scale_res = 0.5, then the image resolution will be reduce by 1/2. 720p
     *becomes 360p.
     *
     *
     * @param rgb_img pass by reference for zero-copy. This data will
     * manipulated by this function.
     **************************************************************************/
    void crop_n_resize(Mat &rgb_img);

    /**************************************************************************
     * @brief This function will update the binary mask and the filter binary
     * mask class member variables (this->bin_mask & this->filter_bin_mask).
     * It convert the rgb image to hsv color space. Then creates a yellow mask
     * and white mask. combined the them together, then filters the results.
     * @note The member variables that are updated by this function:
     *    * bin_mask
     *    * filter_bin_mask
     *    * hsv_img
     *    * white_mask
     *    * yellow_mask
     *
     * @param rgb_img The RGB Image is zero-copied but its value will not be
     * changed
     *************************************************************************/
    void lane_mask(const Mat &rgb_img);

    /**************************************************************************
     * @brief This function will update the binary mask and the filter binary
     * mask (bin_mask & filter_bin_mask).
     * It filters out objects in the image base on area they take up. Used the
     * configuration setting min_area and max_area.
     * @note The member variables that are updated by this function:
     *    * bin_mask
     *    * filter_bin_mask
     **************************************************************************/
    void filter_area();

   public:
    // ============== Constructor(s)/Destructor(s) ============================
    /**
     * @brief Construct a new Ld_preprocessor object
     *
     * @param config ld_config object containing the configuration settings
     */
    Ld_preprocessor(Ld_config &config) : config(make_unique<Ld_config>(config)) {
        // make sure the opencv is using optimize settings
        cv::setUseOptimized(true);
    };

    // ============== Main interface Function(s) ==============================
    /**************************************************************************
     *@brief This will run one preprocessing loop. (crop, resize, color mask,
     * filtering). Internal variable are updated in this function.
     * After this function runs. The bin_mask and filter_bin_mask are ready
     * to be used for tht rest of the pipeline.
     * @note The member variables that are updated by this function:
     *    * bin_mask
     *    * filter_bin_mask
     *    * hsv_img
     *    * white_mask
     *    * yellow_mask
     *
     * @param rgb_img updated to ROI and resize.
     **************************************************************************/
    void run(Mat &rgb_img);

    // ============== Getters/Setter Function(s) ===============================
    Mat &get_bin_mask() { return bin_mask; }
    Mat &get_filter_bin_mask() { return filter_bin_mask; }
};
}  // namespace preprocessor_ns
#endif