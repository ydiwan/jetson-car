#include "vehicle_hardware/JetsonCarSystemHardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Linux hardware i/o
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <cmath>
#include <algorithm> 

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

  // Load parameters
  steering_serial_port_ = info_.hardware_parameters["steering_serial_port"];
  pwm_left_pin_ = std::stoi(info_.hardware_parameters["pwm_left_pin"]);
  pwm_right_pin_ = std::stoi(info_.hardware_parameters["pwm_right_pin"]);

  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Hardware initialized in Open-Loop mode.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JetsonCarSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Rear Wheels
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "rl_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_rl_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "rl_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rl_wheel_vel_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "rr_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_rr_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "rr_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rr_wheel_vel_));

  // Front Steering
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "fl_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fl_steering_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "fr_steering_joint", hardware_interface::HW_IF_POSITION, &hw_fr_steering_pos_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JetsonCarSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Rear Wheels expect Velocity 
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "rl_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rl_wheel_cmd_vel_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "rr_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rr_wheel_cmd_vel_));

  // Front Steering expects Position 
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

  maestro_fd_ = open_maestro_serial(steering_serial_port_);

  // Initialize commands to 0
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

  set_motor_pwm(pwm_left_pin_, 0.0);
  set_motor_pwm(pwm_right_pin_, 0.0);
  
  // Center the steering
  set_maestro_target(0, 0.0);

  if (maestro_fd_ != -1) {
  ::close(maestro_fd_);
  maestro_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type JetsonCarSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
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
  // Push actual commands to hardware
    set_motor_pwm(pwm_left_pin_, hw_rl_wheel_cmd_vel_);
    set_motor_pwm(pwm_right_pin_, hw_rr_wheel_cmd_vel_);
    set_maestro_target(0, hw_fl_steering_cmd_pos_);

  return hardware_interface::return_type::OK;
}

bool JetsonCarSystemHardware::open_maestro_serial(const std::string & port)
{
  // Open the serial port for Read/Write, no controlling terminal
  maestro_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (maestro_fd_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("JetsonCarSystemHardware"), "Failed to open Maestro on %s", port.c_str());
    return false;
  }

  // Configure standard Linux Serial settings (termios) for Pololu
  struct termios options;
  tcgetattr(maestro_fd_, &options);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
  options.c_oflag &= ~OPOST;                          // Raw output
  tcsetattr(maestro_fd_, TCSANOW, &options);

  return true;
}

void JetsonCarSystemHardware::set_maestro_target(int channel, double angle_rad)
{
  if (maestro_fd_ == -1) return;

  // Translate Radians to PWM
  // 0 rad = Center = 1500 us
  // +/- 0.785 rad = +/- 500 us (1000 us to 2000 us range)
  double pulse_us = 1500.0 - ((angle_rad / 0.785) * 500.0);

  // Clamp values to protect the steering linkages
  pulse_us = std::clamp(pulse_us, 1000.0, 2000.0);

  // The Maestro protocol expects the target in quarter-microseconds
  int target = static_cast<int>(pulse_us * 4.0);

  // Construct the Pololu "Set Target" serial payload (0x84)
  unsigned char command[] = {
    0x84, 
    static_cast<unsigned char>(channel), 
    static_cast<unsigned char>(target & 0x7F), 
    static_cast<unsigned char>((target >> 7) & 0x7F)
  };

  // Write directly to the USB serial bus
  ::write(maestro_fd_, command, sizeof(command));
}

void JetsonCarSystemHardware::set_motor_pwm(int pin, double velocity_rad_s)
{
  // Rad/s to Duty Cycle %
  double max_rad_s = 25.0; // Arbitrary scaler for max RPM
  double speed_percent = std::abs(velocity_rad_s) / max_rad_s;
  speed_percent = std::clamp(speed_percent, 0.0, 1.0);

  // Active-Low Hardware Logic
  double active_low_duty = 1.0 - speed_percent;

  // Write to Jetson Sysfs
  // Standard path: /sys/class/pwm/pwmchip0/pwm0/duty_cycle
  int period_ns = 1000000;
  int duty_ns = static_cast<int>(active_low_duty * period_ns);

  // Determine the correct sysfs path based on the pin.
  std::string pwm_path = "";
  if (pin == 15) pwm_path = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle"; 
  if (pin == 32) pwm_path = "/sys/class/pwm/pwmchip0/pwm2/duty_cycle";

  if (!pwm_path.empty()) {
    std::ofstream duty_file(pwm_path);
    if (duty_file.is_open()) {
      duty_file << duty_ns;
      duty_file.close();
    }
  }
}
}  // namespace vehicle_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vehicle_hardware::JetsonCarSystemHardware, hardware_interface::SystemInterface)