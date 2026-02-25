#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vehicle_controller/waypoint_loader.hpp"

using namespace std::chrono_literals;

class WaypointLoaderNode : public rclcpp::Node
{
   public:
    WaypointLoaderNode() : Node("waypoint_loader")
    {
        // Declare parameters
        this->declare_parameter("csv_file", "");
        this->declare_parameter("publish_frequency", 10.0);  // Hz
        this->declare_parameter("frame_id", "map");

        // Get parameters
        std::string csv_file = this->get_parameter("csv_file").as_string();
        double publish_frequency = this->get_parameter("publish_frequency").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Create publishers
        waypoints_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("waypoints", 10);

        waypoint_names_publisher_ = this->create_publisher<std_msgs::msg::String>("waypoint_names", 10);

        // Create waypoint loader
        waypoint_loader_ = std::make_unique<WaypointLoader>(this->get_logger());

        if (!csv_file.empty())
        {
            waypoints_ = waypoint_loader_->load_waypoints(csv_file);

            if (!waypoints_.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(), csv_file.c_str());

                // Create timer to publish waypoints
                timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency)),
                                                 std::bind(&WaypointLoaderNode::publish_waypoints, this));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "No waypoints loaded from %s", csv_file.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No CSV file specified. Use the 'csv_file' parameter.");
        }
    }

   private:
    std::unique_ptr<WaypointLoader> waypoint_loader_;
    std::vector<Waypoint> waypoints_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_names_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id_;

    void publish_waypoints()
    {
        if (waypoints_.empty())
        {
            return;
        }

        // Create PoseArray message
        auto pose_array_msg = std::make_unique<geometry_msgs::msg::PoseArray>();
        pose_array_msg->header.stamp = this->now();
        pose_array_msg->header.frame_id = frame_id_;

        // Create names message
        auto names_msg = std::make_unique<std_msgs::msg::String>();
        std::string names_str;

        // Add all waypoints to the pose array
        for (const auto &waypoint : waypoints_)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = waypoint.x;
            pose.position.y = waypoint.y;
            pose.position.z = 0.0;

            // Set orientation to identity quaternion (no rotation)
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;

            pose_array_msg->poses.push_back(pose);

            // Add name to names string (comma-separated)
            if (!names_str.empty())
            {
                names_str += ",";
            }
            names_str += waypoint.name.empty() ? "unnamed" : waypoint.name;
        }

        names_msg->data = names_str;

        // Publish messages
        waypoints_publisher_->publish(std::move(pose_array_msg));
        waypoint_names_publisher_->publish(std::move(names_msg));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointLoaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}