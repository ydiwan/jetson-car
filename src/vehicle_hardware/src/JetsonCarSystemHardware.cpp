#include "vehicle_hardware/JetsonCarSystemHardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Placeholder for Jetson GPIO library includes
// #include <JetsonGPIO.h> 

namespace vehicle_hardware
{

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load parameters defined in our <ros2_control> URDF block
  steering_serial_port_ = info_.hardware_parameters["steering_serial_port"];
  pwm_left_pin_ = std::stoi(info_.hardware_parameters["pwm_left_pin"]);
  pwm_right_pin_ = std::stoi(info_.hardware_parameters["pwm_right_pin"]);

  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Hardware initialized in Open-Loop mode.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetsonCarSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Rear Wheels (Position & Velocity)
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "rl_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_rl_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "rl_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rl_wheel_vel_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "rr_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_rr_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "rr_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rr_wheel_vel_));

  // Front Steering (Position only)
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "fl_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fl_steering_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "fr_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fr_steering_pos_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JetsonCarSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Rear Wheels expect Velocity commands
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "rl_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rl_wheel_cmd_vel_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "rr_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rr_wheel_cmd_vel_));

  // Front Steering expects Position commands (radians)
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "fl_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fl_steering_cmd_pos_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "fr_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fr_steering_cmd_pos_));

  return command_interfaces;
}

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Activating hardware...");

  // TODO: Open Serial Port to Maestro
  // maestro_fd_ = open_maestro_serial(steering_serial_port_);

  // TODO: Export Jetson GPIO Pins
  // export_gpio_pwm(pwm_left_pin_);
  // export_gpio_pwm(pwm_right_pin_);

  // Initialize commands to 0 to prevent jump on startup
  hw_rl_wheel_cmd_vel_ = 0.0;
  hw_rr_wheel_cmd_vel_ = 0.0;
  hw_fl_steering_cmd_pos_ = 0.0;
  hw_fr_steering_cmd_pos_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Deactivating hardware. STOPPING CAR.");

  // CRITICAL SAFETY: Hardware PRD states 100% duty cycle stops the active-low motors
  // set_motor_pwm(pwm_left_pin_, 100.0);
  // set_motor_pwm(pwm_right_pin_, 100.0);
  
  // Center the steering
  // set_maestro_target(0, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type JetsonCarSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // OPEN-LOOP HACK: Since we have no encoders, we assume the motors 
  // perfectly achieved the velocity we commanded in the last cycle.
  hw_rl_wheel_vel_ = hw_rl_wheel_cmd_vel_;
  hw_rr_wheel_vel_ = hw_rr_wheel_cmd_vel_;

  // Mathematically integrate the fake velocity into a fake position
  hw_rl_wheel_pos_ += hw_rl_wheel_vel_ * period.seconds();
  hw_rr_wheel_pos_ += hw_rr_wheel_vel_ * period.seconds();

  // Steering position feedback loopback
  hw_fl_steering_pos_ = hw_fl_steering_cmd_pos_;
  hw_fr_steering_pos_ = hw_fr_steering_cmd_pos_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JetsonCarSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. Translate commanded Rad/s to Active-Low PWM (0-1000 scale)
  // PRD Note: 0% duty cycle = max speed, 100% = stop
  // double left_pwm = math_conversion(hw_rl_wheel_cmd_vel_);
  // double right_pwm = math_conversion(hw_rr_wheel_cmd_vel_);
  
  // set_motor_pwm(pwm_left_pin_, left_pwm);
  // set_motor_pwm(pwm_right_pin_, right_pwm);

  // 2. Translate commanded Radians to Pololu Maestro Target
  // set_maestro_target(0, hw_fl_steering_cmd_pos_);

  return hardware_interface::return_type::OK;
}

}  // namespace vehicle_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vehicle_hardware::JetsonCarSystemHardware, hardware_interface::SystemInterface)