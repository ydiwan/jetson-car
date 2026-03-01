/**
 * @file ld_config.h
 * @brief This file contains the header file for struct that contain and setups
 * variables for configuring the Lane detection algrothrim for the lane_detector
 * class.
 *
 * @author Trevin Vaughan
 * @date 2025-07-28
 *
 * @par License:
 *      MIT License
 *
 * @par Description:
 *      - a struct with public member to easily access for the lane detection
 * class
 *
 * @par Requirements:
 *      - OpenCV >= 4.5.0
 *      - C++17 or higher
 *      - ROS2 Humble
 *
 * @par Changelog:
 *      - 2025-07-28: Initial implementation
 */
#ifndef LD_CONFIG
#define LD_CONFIG

#include <opencv2/core/types.hpp>
#include <rclcpp/node.hpp>

/** @namespace ld_config_ns namespace need for names rom other classes
 */
namespace ld_config_ns {

// opencv names
using cv::Point2f;

// std names
using std::vector;

/**
 * @struct Ld_config
 * @brief stuct containing lane detection configuration variables to adjust
 * the lane detection algrothrim in the lane_detector class.
 *
 * @example
 * @code{.cpp}
 *      //inside the ros2 node
 *      Ld_config config(this); // passing the ros2 node reference to the
 * constructor
 *
 * @endcode
 */
struct Ld_config {
    int scale_height;           ///< how to much scale input image the height
    double scale_res;           ///< how to scale resolution of input image
    int rgb_img_width;          ///< the width of the RGB of road
    double steps;               ///< how steps to skip in lane_scan() for loop
    int min_area;               ///< minium area for lane line
    int max_area;               ///< maximum area for lane line
    double center_offset;       ///< camera's offset from the center of the road
    int min_lane_width;         ///< minium pixel width of lane line
    int max_lane_width;         ///< maximum pixel width of lane line
    int min_road_width;         ///< minium pixel width of the road
    int max_road_width;         ///< maximum pixel width of the road
    int median_window;          ///< length of sliding median filter window
    double median_threshold;    ///< threshold of median filter
    double pos_conf_threshold;  ///< threshold of position confidence
    Point2f top_left_src;       ///< top left source point for bev_trans()
    Point2f top_right_src;      ///< top right source point for bev_trans()
    Point2f btm_left_src;       ///< bottom left source point for bev_trans()
    Point2f btm_right_src;      ///< bottom right source point for bev_trans()

    /**
     * @brief Constructs configuration variables. Use a ros2 node to get the
     * parameters
     * @param node ROS2 Node where you are using the lane detector class
     */
    Ld_config(rclcpp::Node *node) {
        // declare ros2 parameters
        node->declare_parameter("scale_height", 3);
        node->declare_parameter("scale_res", 0.3);
        node->declare_parameter("input_img_width", 640);
        node->declare_parameter("steps", 5);
        node->declare_parameter("min_area", 1);
        node->declare_parameter("max_area", 300);
        node->declare_parameter("min_lane_width", 3);
        node->declare_parameter("max_lane_width", 10);
        node->declare_parameter("min_road_width", 20);
        node->declare_parameter("max_road_width", 60);
        node->declare_parameter("center_offset", 0.1);
        node->declare_parameter("median_window", 8);
        node->declare_parameter("median_threshold", 15.0);
        node->declare_parameter("position_conf_threshold", 0.1);

        // Bird-Eye View transformation parameters
        node->declare_parameter("top_left_src",
                                vector<float>{100.0, 100.0});  // top left source point
        node->declare_parameter("top_right_src",
                                vector<float>{100.0, 0.0});  // top right source point
        node->declare_parameter("btm_left_src",
                                vector<float>{0.0, 100.0});  // bottom right source point
        node->declare_parameter("btm_right_src",
                                vector<float>{0.0, 0.0});  // bottom right source point

        // get parameters and store the results
        scale_height = node->get_parameter("scale_height").as_int();
        scale_res = node->get_parameter("scale_res").as_double();
        rgb_img_width = node->get_parameter("input_img_width").as_int();
        steps = node->get_parameter("steps").as_int();
        min_area = node->get_parameter("min_area").as_int();
        max_area = node->get_parameter("max_area").as_int();
        min_lane_width = node->get_parameter("min_lane_width").as_int();
        max_lane_width = node->get_parameter("max_lane_width").as_int();
        min_road_width = node->get_parameter("min_road_width").as_int();
        max_road_width = node->get_parameter("max_road_width").as_int();
        center_offset = node->get_parameter("center_offset").as_double();
        median_window = node->get_parameter("median_window").as_int();
        median_threshold = node->get_parameter("median_threshold").as_double();

        // need to first get vector<double>
        vector<double> top_left_double =
            node->get_parameter("top_left_src").as_double_array();
        vector<double> top_right_double =
            node->get_parameter("top_right_src").as_double_array();
        vector<double> btm_left_double =
            node->get_parameter("btm_left_src").as_double_array();
        vector<double> btm_right_double =
            node->get_parameter("btm_right_src").as_double_array();

        // convert to cv::Point2f
        top_left_src = Point2f(top_left_double[0], top_left_double[1]);
        top_right_src = Point2f(top_right_double[0], top_right_double[1]);
        btm_left_src = Point2f(btm_left_double[0], btm_left_double[1]);
        btm_right_src = Point2f(btm_right_double[0], btm_right_double[1]);

        // info log parameters
        RCLCPP_INFO(node->get_logger(), " The Video Input width will be: %d",
                    rgb_img_width);
        RCLCPP_INFO(node->get_logger(), " param info: scale resolution by: %f",
                    scale_res);
        RCLCPP_INFO(node->get_logger(), " Get the lower 1/%d of the image ",
                    scale_height);
        RCLCPP_INFO(node->get_logger(),
                    " Min and Max for pixel area for lane will be: %d and %d", min_area,
                    max_area);
        RCLCPP_INFO(node->get_logger(),
                    " Min and Max for pixel width for lane will be: %d and %d",
                    min_lane_width, max_lane_width);
        RCLCPP_INFO(node->get_logger(),
                    " Min and Max for pixel width for road will be: %d and %d",
                    min_road_width, max_road_width);
        RCLCPP_INFO(node->get_logger(),
                    " The center of the image will be offset by: %f ", center_offset);
        RCLCPP_INFO(node->get_logger(),
                    " The window and threshold for median filtering of center "
                    "points will : %d and %f ",
                    median_window, median_threshold);
        RCLCPP_INFO(node->get_logger(),
                    "Bird's Eye View source points = { (%.2f,%.2f), (%.2f,%.2f),\n"
                    "                                  (%.2f,%.2f), (%.2f,%.2f)}",
                    top_left_src.x, top_left_src.y, top_right_src.x, top_right_src.y,
                    btm_left_src.x, btm_left_src.y, btm_right_src.x, btm_right_src.y);
    }
};
}  // namespace ld_config_ns

#endif