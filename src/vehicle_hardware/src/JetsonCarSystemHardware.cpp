#include "vehicle_hardware/JetsonCarSystemHardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <gpiod.h>
#include <cstdlib>

namespace vehicle_hardware
{

class SysfsPWM {
    std::string pwm_path_;
    int chip_idx_ = -1;

public:
    SysfsPWM(int board_pin) {
        std::string address;
        if (board_pin == 32) address = "32e0000.pwm";
        else if (board_pin == 33) address = "32c0000.pwm";
        else if (board_pin == 15) address = "3280000.pwm";
        else return;

        for (int i = 0; i < 10; ++i) {
            char path[256], target[256];
            snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/device", i);
            ssize_t len = readlink(path, target, sizeof(target)-1);
            if (len != -1) {
                target[len] = '\0';
                if (std::string(target).find(address) != std::string::npos) {
                    chip_idx_ = i;
                    break;
                }
            }
        }

        if (chip_idx_ != -1) {
            pwm_path_ = "/sys/class/pwm/pwmchip" + std::to_string(chip_idx_) + "/pwm0";
            
            // Export PWM channel
            std::ofstream exp("/sys/class/pwm/pwmchip" + std::to_string(chip_idx_) + "/export");
            if (exp.is_open()) { exp << 0 << std::endl; exp.close(); }
            
            // Wait for Linux to create folder
            usleep(200000); 

            // Force 777 permissions, allows direct communication with pwm folders
            std::string chmod_cmd = "chmod -R 777 /sys/class/pwm/pwmchip" + std::to_string(chip_idx_);
            int res = system(chmod_cmd.c_str());
            (void)res; // Suppress unused warning

            // Set period to 1kHz
            std::ofstream per(pwm_path_ + "/period");
            if (per.is_open()) { per << 1000000 << std::endl; per.close(); }
            else { RCLCPP_ERROR(rclcpp::get_logger("JetsonCarSystemHardware"), "Still locked out of period for Pin %d!", board_pin); }
            
            // Enable channel
            std::ofstream en(pwm_path_ + "/enable");
            if (en.is_open()) { en << 1 << std::endl; en.close(); }
            
            RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Mapped Pin %d to pwmchip%d with 777 override.", board_pin, chip_idx_);
        }
    }

    void set_duty(double percent) {
        if (chip_idx_ == -1) return;
        int duty_ns = (percent / 100.0) * 1000000; 
        std::ofstream duty(pwm_path_ + "/duty_cycle");
        if (duty.is_open()) { duty << duty_ns << std::endl; }
    }
};

std::unique_ptr<SysfsPWM> pwm_left_obj_;
std::unique_ptr<SysfsPWM> pwm_right_obj_;
struct gpiod_chip *gpio_chip_ = nullptr;
struct gpiod_line *dir_l_line_ = nullptr;
struct gpiod_line *dir_r_line_ = nullptr;

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
  steering_serial_port_ = info_.hardware_parameters["steering_serial_port"];
  pwm_left_pin_ = std::stoi(info_.hardware_parameters["pwm_left_pin"]);
  pwm_right_pin_ = std::stoi(info_.hardware_parameters["pwm_right_pin"]);
  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Hardware initialized in Pure C++ Mode.");
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

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Activating native Kernel drivers...");
  
  if (steering_serial_port_.empty()) {
      steering_serial_port_ = "/dev/ttyACM0"; // Fallback
  }
  
  if (!open_maestro_serial(steering_serial_port_)) {
      RCLCPP_ERROR(rclcpp::get_logger("JetsonCarSystemHardware"), "FATAL: Could not open Maestro USB on %s", steering_serial_port_.c_str());
  } else {
      RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Maestro Connected on %s (FD: %d)", steering_serial_port_.c_str(), maestro_fd_);
  }

  gpio_chip_ = gpiod_chip_open_by_name("gpiochip0");
  if (gpio_chip_) {
      dir_l_line_ = gpiod_chip_get_line(gpio_chip_, 144); // Pin 7
      dir_r_line_ = gpiod_chip_get_line(gpio_chip_, 106); // Pin 31
      gpiod_line_request_output(dir_l_line_, "ros2_control", 1);
      gpiod_line_request_output(dir_r_line_, "ros2_control", 1);
  }

  pwm_right_obj_ = std::make_unique<SysfsPWM>(pwm_right_pin_);
  pwm_left_obj_ = std::make_unique<SysfsPWM>(pwm_left_pin_);

  // Active-low
  if (pwm_right_obj_) pwm_right_obj_->set_duty(100.0);
  if (pwm_left_obj_) pwm_left_obj_->set_duty(100.0);

  hw_rl_wheel_cmd_vel_ = 0.0; hw_rr_wheel_cmd_vel_ = 0.0;
  hw_fl_steering_cmd_pos_ = 0.0; hw_fr_steering_cmd_pos_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Deactivating hardware. STOPPING CAR.");
  if (pwm_left_obj_) pwm_left_obj_->set_duty(100.0);
  if (pwm_right_obj_) pwm_right_obj_->set_duty(100.0);
  
  if (dir_l_line_) gpiod_line_release(dir_l_line_);
  if (dir_r_line_) gpiod_line_release(dir_r_line_);
  if (gpio_chip_) gpiod_chip_close(gpio_chip_);

  set_maestro_target(0, 0.0);
  if (maestro_fd_ != -1) { ::close(maestro_fd_); maestro_fd_ = -1; }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type JetsonCarSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  hw_rl_wheel_vel_ = hw_rl_wheel_cmd_vel_;
  hw_rr_wheel_vel_ = hw_rr_wheel_cmd_vel_;
  hw_rl_wheel_pos_ += hw_rl_wheel_vel_ * period.seconds();
  hw_rr_wheel_pos_ += hw_rr_wheel_vel_ * period.seconds();
  hw_fl_steering_pos_ = hw_fl_steering_cmd_pos_;
  hw_fr_steering_pos_ = hw_fr_steering_cmd_pos_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JetsonCarSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double max_rad_s = 25.0; 
  static bool motors_enabled_ = false;
  static double last_steering_ = -999.0;

  // Maestro Dynamic Kill-Switch
  if (std::abs(hw_rl_wheel_cmd_vel_) < 0.01 && std::abs(hw_rr_wheel_cmd_vel_) < 0.01) {
      if (motors_enabled_) {
          set_maestro_raw(1, 0); 
          set_maestro_raw(2, 0); 
          motors_enabled_ = false;
      }
      if (pwm_left_obj_) pwm_left_obj_->set_duty(100.0);  // Active-low
      if (pwm_right_obj_) pwm_right_obj_->set_duty(100.0); 
  } else {
      if (!motors_enabled_) {
          set_maestro_raw(1, 7000); 
          set_maestro_raw(2, 7000); 
          motors_enabled_ = true;
      }

      // Left wheel
      double speed_l = hw_rl_wheel_cmd_vel_ / max_rad_s;
      if (dir_l_line_) gpiod_line_set_value(dir_l_line_, speed_l < 0 ? 0 : 1);
      double effort_l = std::clamp(std::abs(speed_l) * 100.0, 0.0, 100.0);
      if (pwm_left_obj_) pwm_left_obj_->set_duty(100.0 - effort_l);

      // Right wheel
      double speed_r = hw_rr_wheel_cmd_vel_ / max_rad_s;
      if (dir_r_line_) gpiod_line_set_value(dir_r_line_, speed_r < 0 ? 0 : 1);
      double effort_r = std::clamp(std::abs(speed_r) * 100.0, 0.0, 100.0);
      if (pwm_right_obj_) pwm_right_obj_->set_duty(100.0 - effort_r);
  }

  // Steering
  if (std::abs(hw_fl_steering_cmd_pos_ - last_steering_) > 0.001) {
      set_maestro_target(0, hw_fl_steering_cmd_pos_);
      last_steering_ = hw_fl_steering_cmd_pos_;
  }

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
  unsigned char command[] = { 0x84, static_cast<unsigned char>(channel), static_cast<unsigned char>(target_q_us & 0x7F), static_cast<unsigned char>((target_q_us >> 7) & 0x7F) };
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