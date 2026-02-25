/**
 * @file ld_scanner.h
 * @brief This file contains the implementation file for Ld_scanner class for
 scanning for lane lines segments in a image.
 *
 * @author Trevin Vaughan
 * @date 2025-07-28
 *
 * @par License:
 *      MIT License
 *
 * @par Description:
 *      - This class takes an binary image and scans for lane line segments base
 *        on it width and position in the image.
 *      - Does a median filter on the scan results
 *
 * @par Requirements:
 *      - OpenCV >= 4.5.0
 *      - C++17 or higher
 *      - ROS2 Humble
 *
 * @par Changelog:
 *      - 2025-07-31: Initial creation
 */

#include "../include/lane_detection/ld_scanner.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ld_scanner_ns {
// ============== Adding Names ==============================================
// std naming
using std::max;
using std::min;
using std::pair;

// ============== Main Interface Function Implementations =====================
bool Ld_scanner::run(const Mat &bin_mask) {
    // reset the lane pixel coordinate vectors
    this->left_lane.clear();
    this->right_lane.clear();
    this->center_lane.clear();

    for (int y = 0; y < bin_mask.rows; y += config.steps) {
        vector<pair<int, int>> lane_segments;

        bool in_lane = false;    // Is the index inside a lane segment?
        int segment_start = -1;  // lane segment start x coordinate

        // Find all white pixel (lane markings) in this row
        for (int x = 0; x < bin_mask.cols; x++) {
            // opencv stores black and white image as black = 0 and white =
            // 255.
            uchar pixel_value = bin_mask.at<uchar>(y, x);
            if (lane_segments.size() == 2) {
                // only need need two lane segments, the left and the right
                break;
            }

            if (pixel_value > 0 && !in_lane) {
                // Found a white pixel and we are not inside a lane segment yet.
                // Start of a lane segment then.
                segment_start = x;
                in_lane = true;
            } else if (pixel_value == 0 && in_lane) {
                // transitioning to end of lane segment
                int lane_width = x - segment_start;
                if (lane_width >= config.min_lane_width &&
                    lane_width <= config.max_lane_width) {
                    // End of vail lane segment
                    lane_segments.push_back({segment_start, x - 1});
                    in_lane = false;

                    RCLCPP_DEBUG(get_logger(), "Add Lane Segment that is %d pixels wide",
                                 lane_width);
                } else {
                    // end of the object that is not a lane
                    in_lane = false;
                }
            }
        }

        // Handle case where lane extends to the edge of the image
        if (in_lane && segment_start >= 0) {
            int lane_width = bin_mask.cols - segment_start;
            if (lane_width >= config.min_lane_width &&
                lane_width <= config.max_lane_width) {
                lane_segments.push_back({segment_start, bin_mask.cols - 1});
                RCLCPP_DEBUG(get_logger(), "Add Lane Segment that is %d pixels wide",
                             lane_width);
            }
        }

        // Process lane segments
        if (lane_segments.size() >= 2) {
            // Fix variable assignments: get right edge of left lane and left edge of
            // right lane
            int left_lane_right_edge =
                lane_segments[0].second;  // Right edge of left lane
            int right_lane_left_edge =
                lane_segments.back().first;  // Left edge of right lane
            int road_width = abs(right_lane_left_edge - left_lane_right_edge);
            RCLCPP_DEBUG(get_logger(), "Road width: %d", road_width);

            // check if the road width makes sense
            if (road_width >= config.min_road_width &&
                road_width <= config.max_road_width) {
                // Calculate center between the inner edges of the two lanes

                //  find the mid point between the left and right lane.
                int center_x = (left_lane_right_edge + right_lane_left_edge) / 2;
                this->center_lane.push_back(Point(center_x, y));

                // save left and right points
                this->right_lane.push_back(Point(right_lane_left_edge, y));
                this->left_lane.push_back(Point(left_lane_right_edge, y));

                RCLCPP_DEBUG(
                    get_logger(), "Left, center, right: (%d,%d) (%d,%d) (%d,%d)",
                    left_lane_right_edge, y, right_lane_left_edge, y, center_x, y);
            }
        }

        // do a median filter to remove outlier points
        median_filter();
    }

    // check if we found enough points
    if (this->left_lane.size() > 3) {
        // we found
        return true;
    } else {
        // we found nothing
        return false;
    }
}

// ============== Private Member Function Implementations =====================
void Ld_scanner::median_filter() {
    vector<Point> filtered_center;
    vector<Point> filtered_right;
    vector<Point> filtered_left;

    // get configuration settings for median filtering
    int window_size = config.median_window;
    double threshold = config.median_threshold;

    // go through ever element
    for (size_t i = 0; i < this->center_lane.size(); i++) {
        // calculate current window start and end index
        int start = max(0, (int)i - window_size / 2);
        int end = min((int)center_lane.size() - 1, (int)i + window_size / 2);

        vector<int> window_values;
        for (int j = start; j <= end; j++) {
            if ((size_t)j != i) {
                window_values.push_back(center_lane[j].x);
            }
        }

        // calculated median
        if (!window_values.empty()) {
            auto beginning_iter = window_values.begin();
            auto ending_iter = window_values.end();

            // sort the values in asending order
            sort(beginning_iter, ending_iter);

            // now that is sorted, the middle index is going to be the
            // median
            int median_val = window_values[window_values.size() / 2];

            if (abs(center_lane[i].x - median_val) <= threshold) {
                // check if the current index is within the threshold
                filtered_center.push_back(center_lane[i]);
                filtered_left.push_back(this->left_lane[i]);
                filtered_right.push_back(this->right_lane[i]);
            }
        } else {
            // Keep first/last points if no neighbors
            filtered_center.push_back(center_lane[i]);
            filtered_left.push_back(this->left_lane[i]);
            filtered_right.push_back(this->right_lane[i]);
        }
    }

    if (filtered_center.size() > 0) {
        // assign the left, right, center lanes new filter values
        this->left_lane = filtered_left;
        this->right_lane = filtered_right;
        this->center_lane = filtered_center;
    } else {
        RCLCPP_DEBUG(get_logger(), "The scanner found not vail lanes");
    }
}

}  // namespace ld_scanner_ns