#include <tf2/LinearMath/Quaternion.h>  //need for the angle/orentation

#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  //need for the angle/orentation

class ViconSubscriber : public rclcpp::Node
{
   public:
    ViconSubscriber() : Node("vicon_display_node")
    {
        // Create a QoS profile optimized for low latency
        // This matches what the publisher uses
        auto qos = rclcpp::SensorDataQoS();

        // Create subscription with minimal callback overhead
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "vicon_pose",
            qos,
            std::bind(
                &ViconSubscriber::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Vicon position subscriber started");

        // Initialize timing variables for latency measurement
        last_message_time_ = std::chrono::high_resolution_clock::now();
    }

   private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::chrono::high_resolution_clock::time_point last_message_time_;

    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Get current time for latency calculation
        auto current_time = std::chrono::high_resolution_clock::now();

        // Extract position data - this is the fastest way to access it
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;

        // get the angle for message
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.orientation, quat);

        // convert from quat to eular angles
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // convert for radian to degeee
        roll = roll * 180 / M_PI;
        pitch = pitch * 180 / M_PI;
        yaw = yaw * 180 / M_PI;

        // Print position data with minimal formatting
        // Using printf is faster than std::cout for simple output
        printf(
            "Position: %8.2f %8.2f %8.2f | Orientation (deg): Roll=%8.3f "
            "Pitch=%8.3f Yaw=%8.3f\n",
            x * 1000,
            y * 1000,
            z * 1000,
            roll,
            pitch,
            yaw);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create the subscriber node
    auto node = std::make_shared<ViconSubscriber>();

    // Spin the node - this keeps it running and processing callbacks
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}