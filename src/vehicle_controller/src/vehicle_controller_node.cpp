#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64.hpp>

#include "std_msgs/msg/int32.hpp"
#include "vehicle_controller/vehicle.h"

namespace {
// rename disgusting ros naming convention -------------
// ros2 int stuff
using int_msg = std_msgs::msg::Int32;
using int_sub = rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr;
using int_pub = rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr;
using int_ptr = std_msgs::msg::Int32::SharedPtr;

// ros2 int stuff
using double_msg = std_msgs::msg::Float64;
using double_sub = rclcpp::Subscription<double_msg>::SharedPtr;

// ros2 pose stuff
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using poseStamped_sub = rclcpp::Subscription<PoseStamped>::SharedPtr;
using poseArray_sub = rclcpp::Subscription<PoseArray>::SharedPtr;
// end of renaming ----------------------------------------

// adding ros naming
using rclcpp::init;
using rclcpp::QoS;
using rclcpp::shutdown;
using rclcpp::spin;
using timer_ptr = rclcpp::TimerBase::SharedPtr;

// adding std naming
using std::atomic;
using std::bind;
using namespace std::placeholders;
using std::cout;
using std::endl;
using std::make_pair;
using std::make_shared;
using std::string;
using std::unique_ptr;
using std::chrono::milliseconds;

// adding vehicle names
using vehicle_ns::Vehicle;
}  // namespace

class VehicleControllerNode : public rclcpp::Node {
   public:
    VehicleControllerNode() : Node("vehicle_controller") {
        // declaring parameters
        this->declare_parameter("vicon_kp", 25.0);
        this->declare_parameter("ld_kp", 5.0);
        this->declare_parameter("alpha", 0.95);
        this->declare_parameter("servo_min", 1100);
        this->declare_parameter("servo_max", 1800);
        this->declare_parameter("servo_usb", "/dev/ttyACM0");

        // get parameters
        double vicon_kp = this->get_parameter("vicon_kp").as_double();
        double ld_kp = this->get_parameter("ld_kp").as_double();
        int alpha = this->get_parameter("alpha").as_double();
        int servo_min = this->get_parameter("servo_min").as_int();
        int servo_max = this->get_parameter("servo_max").as_int();
        string servo_path = this->get_parameter("servo_usb").as_string();

        // create subscriptions
        auto qos = rclcpp::SensorDataQoS();
        vicon_sub_ = this->create_subscription<PoseStamped>(
            "vicon_pose", qos, bind(&VehicleControllerNode::vicon_callback, this, _1));
        waypoints_sub_ = this->create_subscription<PoseArray>(
            "waypoints", qos,
            bind(&VehicleControllerNode::waypoints_callback, this, _1));

        // lane detection subscriptions
        ld_delta_sub_ = this->create_subscription<int_msg>(
            "lane_detect/delta", qos,
            bind(&VehicleControllerNode::ld_delta_callback, this, _1));
        ld_pos_conf_sub_ = this->create_subscription<double_msg>(
            "/lane_detect/position_confidence", qos,
            bind(&VehicleControllerNode::ld_pos_conf_callback, this, _1));
        sym_conf_sub_ = this->create_subscription<double_msg>(
            "/lane_detect/symmetry_confidence", qos,
            bind(&VehicleControllerNode::ld_sym_conf_callback, this, _1));

        // publish pwm values
        l_pwm_pub_ = this->create_publisher<int_msg>("gpio/pwm_left", qos);
        r_pwm_pub_ = this->create_publisher<int_msg>("gpio/pwm_right", qos);

        // create timer for control loop
        timer_ = this->create_wall_timer(
            milliseconds(10), bind(&VehicleControllerNode::control_loop, this));

        car_ = std::make_unique<Vehicle>(vicon_kp, ld_kp, alpha, servo_min, servo_max,
                                         servo_path, r_pwm_pub_, l_pwm_pub_);
        car_->update_speed(800, 800);
        car_->motor_on(true);
    }

    ~VehicleControllerNode() {
        // stop the vehicle before shutting down
        // pi_car_->set_speed(1000, 1000);
        car_->update_steering(1500);
        cout << "Vehicle Controller Node is shutting down" << endl;
    }

   private:
    unique_ptr<Vehicle> car_;  // unique pointer of vehicle object
    int_pub r_pwm_pub_;        // publisher for right motor pwm value
    int_pub l_pwm_pub_;        // publisher for left motor pwm value

    // subscriptions
    poseStamped_sub vicon_sub_;
    poseArray_sub waypoints_sub_;

    // subscription from lane detection
    int_sub ld_delta_sub_;        // delta from lane detection node
    double_sub ld_pos_conf_sub_;  // position confidence from lane detection node
    double_sub sym_conf_sub_;     // symmetry confidence from lane detection node

    int ld_delta_;  // stop race condition

    // timer
    timer_ptr timer_;

    // waypoints
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_target_idx_ = 0;

    void vicon_callback(const PoseStamped::SharedPtr msg) {
        car_->vicon_update_pose(msg->pose);
    }

    void waypoints_callback(const PoseArray::SharedPtr msg) {
        waypoints_.clear();
        for (const auto &pose : msg->poses)  // loading all waypoint received
        {
            double x = pose.position.x;
            double y = pose.position.y;
            waypoints_.push_back(make_pair(x, y));
        }
    }

    void ld_delta_callback(int_ptr msg) { ld_delta_ = msg->data; }
    void ld_pos_conf_callback(double_msg msg) { car_->set_pos_conf(msg.data); }
    void ld_sym_conf_callback(double_msg msg) { car_->set_sym_conf(msg.data); }

    void control_loop() {
        // skip if we have no waypoints or have completed all waypoints
        if (waypoints_.empty() || current_target_idx_ >= waypoints_.size()) {
            RCLCPP_DEBUG(this->get_logger(), "Reached all waypoints, YAY!");
            car_->update_steering(1500); //center the steering
            car_->motor_on(false);       //turn off the motors
            return;
        } else {
            double target_x = waypoints_[current_target_idx_].first;
            double target_y = waypoints_[current_target_idx_].second;

            // calculate distance to target
            double distance = car_->vicon_cal_dis(target_x, target_y);

            // check if close enough to target
            if (distance <= 100) {
                current_target_idx_++;
                target_x = waypoints_[current_target_idx_].first;
                target_y = waypoints_[current_target_idx_].second;
            }

            RCLCPP_DEBUG(this->get_logger(), "Current target; (%f, %f)", target_x,
                         target_y);
            car_->motor_on(true);  // make sure the motor is on =)
            car_->control_step({target_x, target_y}, ld_delta_);
        }
    }
};

int main(int argc, char *argv[]) {
    init(argc, argv);
    auto node = make_shared<VehicleControllerNode>();
    spin(node);
    shutdown();
    return 0;
}