#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace
{
using twist_msg = geometry_msgs::msg::Twist;
using twist_pub = rclcpp::Publisher<twist_msg>::SharedPtr;
using pose_msg = geometry_msgs::msg::PoseStamped;
using pose_msg_ptr = pose_msg::SharedPtr;
using pose_sub = rclcpp::Subscription<pose_msg>::SharedPtr;

using namespace std::placeholders;
using std::bind;
using std::make_shared;
}  // namespace

class Speedometer_Node : public rclcpp::Node
{
   public:
    Speedometer_Node() : Node("speedometer")
    {
        // Better QoS settings

        // get pose msg for vicon reader node
        pose_sub_ = this->create_subscription<pose_msg>(
            "vicon_pose", 2, bind(&Speedometer_Node::cal_velocities, this, _1));

        // velocity publisher
        velocity_pub_ = this->create_publisher<twist_msg>("velocity", 1);

        // Declare parameters for median filter
        this->declare_parameter("median_window_size", 5);
        this->declare_parameter("median_threshold", 2.0);  // rad/s

        median_window_size_ = this->get_parameter("median_window_size").as_int();
        median_threshold_ = this->get_parameter("median_threshold").as_double();

        RCLCPP_INFO(
            this->get_logger(),
            "Speedometer_node has started with median filter (window=%d, threshold=%.2f rad/s)",
            median_window_size_, median_threshold_);
    }

   private:
    pose_sub pose_sub_;
    twist_pub velocity_pub_;
    pose_msg_ptr last_pose_msg;

    // Median filter variables
    std::deque<double> angular_velo_history_;
    int median_window_size_;
    double median_threshold_;

    // the vicon record data at 100hz
    const double VICON_DT = 0.01;  // therefore 0.01 sec period

    double apply_median_filter(double new_value)
    {
        // Add new value to history
        angular_velo_history_.push_back(new_value);

        // Maintain window size
        if (angular_velo_history_.size() > static_cast<size_t>(median_window_size_))
        {
            angular_velo_history_.pop_front();
        }

        // Need at least 3 values for meaningful median
        if (angular_velo_history_.size() < 3)
        {
            return new_value;
        }

        // Calculate median
        std::vector<double> sorted_values(angular_velo_history_.begin(),
                                          angular_velo_history_.end());
        std::sort(sorted_values.begin(), sorted_values.end());

        double median;
        size_t n = sorted_values.size();
        if (n % 2 == 0)
        {
            median = (sorted_values[n / 2 - 1] + sorted_values[n / 2]) / 2.0;
        }
        else
        {
            median = sorted_values[n / 2];
        }

        // Check if new value is within threshold of median
        if (std::abs(new_value - median) > median_threshold_)
        {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Angular velocity %.3f rad/s rejected (median=%.3f, diff=%.3f > threshold=%.3f)",
                new_value, median, std::abs(new_value - median), median_threshold_);
            return median;
        }

        return new_value;
    }

    void cal_velocities(pose_msg_ptr pose_msg)
    {
        double x_velo;        // x velocity
        double y_velo;        // y velocity
        double angular_velo;  // how fast the car is turning left and right
        tf2::Quaternion last_quat;
        tf2::Quaternion current_quat;
        double last_yaw;
        double current_yaw;
        double delta_yaw;
        twist_msg velocity_msg;

        // check if the last_pose_msg is empty
        if (!last_pose_msg)
        {
            last_pose_msg = pose_msg;
            return;  // end early
        }

        // calculate x, y, and z velocities
        x_velo = (pose_msg->pose.position.x - last_pose_msg->pose.position.x) / VICON_DT;
        y_velo = (pose_msg->pose.position.y - last_pose_msg->pose.position.y) / VICON_DT;

        tf2::fromMsg(pose_msg->pose.orientation, current_quat);
        tf2::fromMsg(last_pose_msg->pose.orientation, last_quat);

        // since it's a car I only care about the yaw
        double dont_care;
        tf2::Matrix3x3(current_quat).getRPY(dont_care, dont_care, current_yaw);
        tf2::Matrix3x3(last_quat).getRPY(dont_care, dont_care, last_yaw);

        // get difference
        delta_yaw = current_yaw - last_yaw;

        // we assume the car took the shortest path
        // to reach it current angle.
        if (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;
        if (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;

        angular_velo = delta_yaw / VICON_DT;

        // Apply median filter to angular velocity
        double filtered_angular_velo = apply_median_filter(angular_velo);

        // the vicon angle samples has jitter
        // so round to tenths place
        filtered_angular_velo = round(100 * filtered_angular_velo) / 100;

        // velocity message
        velocity_msg.linear.x = x_velo;
        velocity_msg.linear.y = y_velo;
        velocity_msg.angular.z = filtered_angular_velo;

        velocity_pub_->publish(velocity_msg);

        // update last pose
        last_pose_msg = pose_msg;

        // debug logging
        RCLCPP_DEBUG(this->get_logger(),
                     "X velocity: %.3f m/s, Y velocity: %.3f m/s, Angular velocity: %.3f "
                     "degree/sec (raw: %.3f)",
                     x_velo, y_velo, round(100 * filtered_angular_velo * (180 / M_PI)) / 100,
                     round(100 * angular_velo * (180 / M_PI) / 100));

        RCLCPP_DEBUG(this->get_logger(),
                     "current and last: X: %.3fm, %.3fm, Y: %.3fm, %.3fm, Yaw: %.3f°, %.3f°",
                     pose_msg->pose.position.x, last_pose_msg->pose.position.x,
                     pose_msg->pose.position.y, last_pose_msg->pose.position.y,
                     current_yaw * (180 / M_PI), last_yaw * (180 / M_PI));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<Speedometer_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}