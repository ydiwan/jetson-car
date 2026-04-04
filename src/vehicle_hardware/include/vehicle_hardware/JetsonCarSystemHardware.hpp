#ifndef VEHICLE_HARDWARE__JETSON_CAR_SYSTEM_HARDWARE_HPP_
#define VEHICLE_HARDWARE__JETSON_CAR_SYSTEM_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32.hpp>

namespace vehicle_hardware
{
class JetsonCarSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JetsonCarSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters loaded from URDF
  std::string steering_serial_port_;
  int pwm_left_pin_;
  int pwm_right_pin_;

  // Internal State
  double hw_rl_wheel_pos_ = 0.0;
  double hw_rl_wheel_vel_ = 0.0;
  double hw_rr_wheel_pos_ = 0.0;
  double hw_rr_wheel_vel_ = 0.0;
  double hw_fl_steering_pos_ = 0.0;
  double hw_fr_steering_pos_ = 0.0;

  // Internal Command
  double hw_rl_wheel_cmd_vel_ = 0.0;
  double hw_rr_wheel_cmd_vel_ = 0.0;
  double hw_fl_steering_cmd_pos_ = 0.0;
  double hw_fr_steering_cmd_pos_ = 0.0;

  // Hardware Communication Handles 
  int maestro_fd_ = -1; // File descriptor for USB serial

  // Private IO Helpers 
  bool open_maestro_serial(const std::string & port);
  void set_maestro_target(int channel, double angle_rad);
  void set_maestro_raw(int channel, int target_q_us);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_pwm_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_pwm_pub_;
};
}  // namespace vehicle_hardware

#endif  // VEHICLE_HARDWARE__JETSON_CAR_SYSTEM_HARDWARE_HPP_