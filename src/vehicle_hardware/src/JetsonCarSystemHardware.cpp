#include "vehicle_hardware/JetsonCarSystemHardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <algorithm>

namespace vehicle_hardware
{

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;

  steering_serial_port_ = info_.hardware_parameters["steering_serial_port"];
  
  // Publisher
  node_ = std::make_shared<rclcpp::Node>("cpp_pwm_bridge");
  left_pwm_pub_ = node_->create_publisher<std_msgs::msg::Int32>("gpio/pwm_left", 10);
  right_pwm_pub_ = node_->create_publisher<std_msgs::msg::Int32>("gpio/pwm_right", 10);

  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Hardware initialized in Hybrid Mode.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetsonCarSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface("rl_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_rl_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("rl_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rl_wheel_vel_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("rr_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_rr_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("rr_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rr_wheel_vel_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("fl_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fl_steering_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("fr_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fr_steering_pos_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JetsonCarSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface("rl_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rl_wheel_cmd_vel_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("rr_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rr_wheel_cmd_vel_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("fl_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fl_steering_cmd_pos_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("fr_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fr_steering_cmd_pos_));
  return command_interfaces;
}

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Activating Hybrid C++/Python Drivers...");

  maestro_fd_ = open_maestro_serial(steering_serial_port_);

  hw_rl_wheel_cmd_vel_ = 0.0; hw_rr_wheel_cmd_vel_ = 0.0;
  hw_fl_steering_cmd_pos_ = 0.0; hw_fr_steering_cmd_pos_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Deactivating hardware. STOPPING CAR.");

  std_msgs::msg::Int32 stop_msg;
  stop_msg.data = 1000;
  if(left_pwm_pub_) left_pwm_pub_->publish(stop_msg);
  if(right_pwm_pub_) right_pwm_pub_->publish(stop_msg);

  set_maestro_target(0, 0.0);
  if (maestro_fd_ != -1) { ::close(maestro_fd_); maestro_fd_ = -1; }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type JetsonCarSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  hw_rl_wheel_vel_ = hw_rl_wheel_cmd_vel_;
  hw_rr_wheel_vel_ = hw_rr_wheel_cmd_vel_;
  hw_rl_wheel_pos_ += hw_rl_wheel_vel_ * period.seconds();
  hw_rr_wheel_pos_ += hw_rr_wheel_vel_ * period.seconds();
  hw_fl_steering_pos_ = hw_fl_steering_cmd_pos_;
  hw_fr_steering_pos_ = hw_fr_steering_cmd_pos_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JetsonCarSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double max_rad_s = 25.0; 

  if (std::abs(hw_rl_wheel_cmd_vel_) < 0.01 && std::abs(hw_rr_wheel_cmd_vel_) < 0.01) {
      set_maestro_raw(1, 0); 
      set_maestro_raw(2, 0); 
      
      std_msgs::msg::Int32 stop_msg;
      stop_msg.data = 1000;
      if(left_pwm_pub_) left_pwm_pub_->publish(stop_msg);
      if(right_pwm_pub_) right_pwm_pub_->publish(stop_msg);
  } else {
      set_maestro_raw(1, 7000); 
      set_maestro_raw(2, 7000); 

      // Left wheel
      double speed_l = hw_rl_wheel_cmd_vel_ / max_rad_s;
      int left_pwm = 1000 - static_cast<int>(std::clamp(std::abs(speed_l), 0.0, 1.0) * 1000.0);
      if (speed_l < 0) left_pwm = -left_pwm;

      // Right wheel
      double speed_r = hw_rr_wheel_cmd_vel_ / max_rad_s;
      int right_pwm = 1000 - static_cast<int>(std::clamp(std::abs(speed_r), 0.0, 1.0) * 1000.0);
      if (speed_r < 0) right_pwm = -right_pwm;

      std_msgs::msg::Int32 l_msg, r_msg;
      l_msg.data = left_pwm;
      r_msg.data = right_pwm;
      
      if(left_pwm_pub_) left_pwm_pub_->publish(l_msg);
      if(right_pwm_pub_) right_pwm_pub_->publish(r_msg);
  }

  set_maestro_target(0, hw_fl_steering_cmd_pos_);

  return hardware_interface::return_type::OK;
}

bool JetsonCarSystemHardware::open_maestro_serial(const std::string & port)
{
  maestro_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (maestro_fd_ == -1) return false;
  struct termios options;
  tcgetattr(maestro_fd_, &options);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
  options.c_oflag &= ~OPOST;                          
  tcsetattr(maestro_fd_, TCSANOW, &options);
  return true;
}

void JetsonCarSystemHardware::set_maestro_raw(int channel, int target_q_us)
{
  if (maestro_fd_ == -1) return;
  unsigned char command[] = { 0x84, static_cast<unsigned char>(channel), 
                              static_cast<unsigned char>(target_q_us & 0x7F), 
                              static_cast<unsigned char>((target_q_us >> 7) & 0x7F) };
  ::write(maestro_fd_, command, sizeof(command));
}

void JetsonCarSystemHardware::set_maestro_target(int channel, double angle_rad)
{
  if (maestro_fd_ == -1) return;
  double pulse_us = 1500.0 - ((angle_rad / 0.785) * 500.0);
  pulse_us = std::clamp(pulse_us, 1000.0, 2000.0);
  int target = static_cast<int>(pulse_us * 4.0);
  unsigned char command[] = { 0x84, static_cast<unsigned char>(channel), static_cast<unsigned char>(target & 0x7F), static_cast<unsigned char>((target >> 7) & 0x7F) };
  ::write(maestro_fd_, command, sizeof(command));
}
} // namespace vehicle_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vehicle_hardware::JetsonCarSystemHardware, hardware_interface::SystemInterface)