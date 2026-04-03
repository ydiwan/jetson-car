#include "vehicle_hardware/JetsonCarSystemHardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <JetsonGPIO.h>

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
  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Activating hardware with JetsonGPIO...");

  // Open Maestro Serial
  maestro_fd_ = open_maestro_serial(steering_serial_port_);

  // JetsionGPIO init
  GPIO::setmode(GPIO::BOARD);
  GPIO::setwarnings(false);

  int dir_r = 31;
  int dir_l = 7;
  
  // Setup Direction Pins
  GPIO::setup(dir_r, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(dir_l, GPIO::OUT, GPIO::HIGH);

  // Setup PWM Pins (1 kHz frequency)
  pwm_right_obj_ = std::make_unique<GPIO::PWM>(pwm_right_pin_, 1000);
  pwm_left_obj_ = std::make_unique<GPIO::PWM>(pwm_left_pin_, 1000);

  // Start with 0% duty cycle (STOP)
  pwm_right_obj_->start(0.0);
  pwm_left_obj_->start(0.0);

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
  
  // Stop PWM and cleanup pins safely
  if (pwm_left_obj_) pwm_left_obj_->stop();
  if (pwm_right_obj_) pwm_right_obj_->stop();
  GPIO::cleanup(); 

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
  // Rad/s to Duty Cycle %
  double max_rad_s = 25.0; // Tune this to match top speed

  // Left wheel
  double speed_l = hw_rl_wheel_cmd_vel_ / max_rad_s;
  if (speed_l < 0) {
      GPIO::output(7, GPIO::LOW); // Reverse
  } else {
      GPIO::output(7, GPIO::HIGH); // Forward
  }
  double duty_l = std::clamp(std::abs(speed_l) * 100.0, 0.0, 100.0);
  if (pwm_left_obj_) pwm_left_obj_->ChangeDutyCycle(duty_l);

  // Right wheel
  double speed_r = hw_rr_wheel_cmd_vel_ / max_rad_s;
  if (speed_r < 0) {
      GPIO::output(31, GPIO::LOW); // Reverse
  } else {
      GPIO::output(31, GPIO::HIGH); // Forward
  }
  double duty_r = std::clamp(std::abs(speed_r) * 100.0, 0.0, 100.0);
  if (pwm_right_obj_) pwm_right_obj_->ChangeDutyCycle(duty_r);

  // Steering
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
} // namespace vehicle_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vehicle_hardware::JetsonCarSystemHardware, hardware_interface::SystemInterface)