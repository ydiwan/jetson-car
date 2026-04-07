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
    int duty_fd_ = -1;
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

            std::ofstream exp("/sys/class/pwm/pwmchip" + std::to_string(chip_idx_) + "/export");
            if (exp.is_open()) { exp << 0 << std::endl; exp.close(); }

            usleep(200000);

            std::string chmod_cmd = "chmod -R 777 /sys/class/pwm/pwmchip" + std::to_string(chip_idx_) + " 2>/dev/null";
            int res = system(chmod_cmd.c_str());
            (void)res;

            std::ofstream per(pwm_path_ + "/period");
            if (per.is_open()) { per << 1000000 << std::endl; per.close(); }

            std::ofstream en(pwm_path_ + "/enable");
            if (en.is_open()) { en << 1 << std::endl; en.close(); }

            duty_fd_ = open((pwm_path_ + "/duty_cycle").c_str(), O_WRONLY);
            RCLCPP_INFO(rclcpp::get_logger("JetsonCarSystemHardware"), "Mapped Pin %d to pwmchip%d (RT Safe FD: %d)", board_pin, chip_idx_, duty_fd_);
        }
    }

    ~SysfsPWM() { if (duty_fd_ != -1) close(duty_fd_); }

    void set_duty(double percent) {
        if (duty_fd_ == -1) return;
        int duty_ns = (percent / 100.0) * 1000000;
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "%d\n", duty_ns);
        pwrite(duty_fd_, buf, len, 0);
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
  if (steering_serial_port_.empty()) steering_serial_port_ = "/dev/ttyACM0";

  if (!open_maestro_serial(steering_serial_port_)) {
      RCLCPP_ERROR(rclcpp::get_logger("JetsonCarSystemHardware"), "FATAL: Could not open Maestro USB.");
  }

  gpio_chip_ = gpiod_chip_open_by_name("gpiochip0");
  if (gpio_chip_) {
      dir_l_line_ = gpiod_chip_get_line(gpio_chip_, 144);
      dir_r_line_ = gpiod_chip_get_line(gpio_chip_, 106);
      gpiod_line_request_output(dir_l_line_, "ros2_control", 1);
      gpiod_line_request_output(dir_r_line_, "ros2_control", 1);
  }

  pwm_right_obj_ = std::make_unique<SysfsPWM>(pwm_right_pin_);
  pwm_left_obj_ = std::make_unique<SysfsPWM>(pwm_left_pin_);

  if (pwm_right_obj_) pwm_right_obj_->set_duty(100.0);
  if (pwm_left_obj_) pwm_left_obj_->set_duty(100.0);

  hw_rl_wheel_cmd_vel_ = 0.0; hw_rr_wheel_cmd_vel_ = 0.0;
  hw_fl_steering_cmd_pos_ = 0.0; hw_fr_steering_cmd_pos_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetsonCarSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (pwm_left_obj_) pwm_left_obj_->set_duty(100.0);
  if (pwm_right_obj_) pwm_right_obj_->set_duty(100.0);
  if (dir_l_line_) gpiod_line_release(dir_l_line_);
  if (dir_r_line_) gpiod_line_release(dir_r_line_);
  if (gpio_chip_) gpiod_chip_close(gpio_chip_);
  set_maestro_raw(0, 0);
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
  static double last_effort_l_ = -999.0;
  static double last_effort_r_ = -999.0;

  static int wakeup_delay_ticks_ = 0; // State machine to replace usleep
  static int heartbeat_counter_ = 0;

  if (heartbeat_counter_++ % 100 == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("JetsonCarSystemHardware"), "Loop running. Current cmd_vel: %.2f", hw_rl_wheel_cmd_vel_);
  }

  if (std::abs(hw_rl_wheel_cmd_vel_) < 0.01 && std::abs(hw_rr_wheel_cmd_vel_) < 0.01) {
      if (motors_enabled_) {
          set_maestro_raw(1, 0);
          set_maestro_raw(2, 0);
          motors_enabled_ = false;
      }
      if (std::abs(last_effort_l_ - 0.0) > 0.01) {
          if (pwm_left_obj_) pwm_left_obj_->set_duty(100.0);
          last_effort_l_ = 0.0;
      }
      if (std::abs(last_effort_r_ - 0.0) > 0.01) {
          if (pwm_right_obj_) pwm_right_obj_->set_duty(100.0);
          last_effort_r_ = 0.0;
      }
  } else {
      if (!motors_enabled_) {
          set_maestro_raw(1, 7000);
          set_maestro_raw(2, 7000);
          motors_enabled_ = true;
          wakeup_delay_ticks_ = 3; // 60ms delay
      }

      if (wakeup_delay_ticks_ > 0) {
          wakeup_delay_ticks_--; // Safely count
      } else {
          double speed_l = hw_rl_wheel_cmd_vel_ / max_rad_s;
          if (dir_l_line_) gpiod_line_set_value(dir_l_line_, speed_l < 0 ? 0 : 1);
          double effort_l = std::clamp(std::abs(speed_l) * 100.0, 0.0, 100.0);
          if (std::abs(effort_l - last_effort_l_) > 0.01) {
              if (pwm_left_obj_) pwm_left_obj_->set_duty(100.0 - effort_l);
              last_effort_l_ = effort_l;
          }

          double speed_r = hw_rr_wheel_cmd_vel_ / max_rad_s;
          if (dir_r_line_) gpiod_line_set_value(dir_r_line_, speed_r < 0 ? 0 : 1);
          double effort_r = std::clamp(std::abs(speed_r) * 100.0, 0.0, 100.0);
          if (std::abs(effort_r - last_effort_r_) > 0.01) {
              if (pwm_right_obj_) pwm_right_obj_->set_duty(100.0 - effort_r);
              last_effort_r_ = effort_r;
          }
      }
  }

  if (std::abs(hw_fl_steering_cmd_pos_ - last_steering_) > 0.001) {
      set_maestro_target(0, hw_fl_steering_cmd_pos_);
      last_steering_ = hw_fl_steering_cmd_pos_;
  }

  return hardware_interface::return_type::OK;
}

bool JetsonCarSystemHardware::open_maestro_serial(const std::string & port)
{
  maestro_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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

  double pulse_us = 1500.0;

  if (angle_rad > 0.0) {
      // Turning Left needs higher multiplier (850) to turn left servo same amount to the left
      pulse_us -= (angle_rad / 0.785) * 850.0;
  } else {
      // Turning Right keeps same multipler (500)
      pulse_us -= (angle_rad / 0.785) * 500.0;
  }

  // Widen the clamp (turning limit)
  pulse_us = std::clamp(pulse_us, 800.0, 2200.0);

  int target = static_cast<int>(pulse_us * 4.0);
  unsigned char command[] = { 0x84, static_cast<unsigned char>(channel), static_cast<unsigned char>(target & 0x7F), static_cast<unsigned char>((target >> 7) & 0x7F) };
  ::write(maestro_fd_, command, sizeof(command));
}
} // namespace vehicle_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vehicle_hardware::JetsonCarSystemHardware, hardware_interface::SystemInterface)