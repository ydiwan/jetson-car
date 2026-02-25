#include "vehicle_controller/vehicle.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <rclcpp/logging.hpp>
#include <rclcpp/visibility_control.hpp>

#include "vehicle_controller/msc_if.h"

namespace vehicle_ns {
// ============== Adding Names =========================================================
// adding std naming
using std::clamp;

// ============== Actuators Control Functions ==========================================
void Vehicle::motor_on(bool on) {
    if (on) {
        // turn them on
        sc.set_pin(1, 1);
        sc.set_pin(2, 1);
    } else {
        // turn them off
        sc.set_pin(1, 0);
        sc.set_pin(2, 0);
    }
}

void Vehicle::update_speed() {
    float servo_range = this->servo_max - servo_min;
    float turn_coeff = (this->servo - this->servo_min) / servo_range;

    int left_pwm = 1000 * (1.0 - turn_coeff);
    int right_pwm = 1000 * turn_coeff;

    // Ensure speed is within valid range
    left_pwm = clamp(left_pwm, 0, 900);
    right_pwm = clamp(right_pwm, 0, 900);

    RCLCPP_DEBUG(get_logger(),
                 "Automatic motor pwm values calculation: servo_range = %f, "
                 "turn_coeff = %f, left pwm = %d, right pwm = %d",
                 servo_range, turn_coeff, left_pwm, right_pwm);

    this->left_pwm = left_pwm;
    this->right_pwm = right_pwm;

    // publish value
    auto left_msg = int_msg().set__data(this->left_pwm);
    auto right_msg = int_msg().set__data(this->right_pwm);

    this->left_pwm_pub->publish(left_msg);
    this->right_pwm_pub->publish(right_msg);
}

void Vehicle::update_speed(int left_pwm, int right_pwm) {
    // Ensure speed is within valid range
    left_pwm = clamp(left_pwm, 0, 1000);
    right_pwm = clamp(right_pwm, 0, 1000);

    RCLCPP_DEBUG(get_logger(), "Manually set pwm values: left pwm = %d, right pwm = %d",
                 left_pwm, right_pwm);

    this->left_pwm = left_pwm;
    this->right_pwm = right_pwm;

    // publish value
    auto left_msg = int_msg().set__data(this->left_pwm);
    auto right_msg = int_msg().set__data(this->right_pwm);

    this->left_pwm_pub->publish(left_msg);
    this->right_pwm_pub->publish(right_msg);
}

void Vehicle::update_steering(int servo_value) {
    // constrain servo value between min and max values
    servo_value = clamp(servo_value, this->servo_min, this->servo_max);

    this->servo = servo_value;

    this->sc.set_servo(0, servo_value);
}

// ============== Lane Detection Functions ========================================

void Vehicle::set_pos_conf(double pos_conf) { this->pos_conf = pos_conf; }

void Vehicle::set_sym_conf(double sym_conf) { this->sym_conf = sym_conf; }

int Vehicle::ld_pro_controller(int delta_x) {
    // Calculate the offset from 1500 using the error (delta_x)
    int offset = this->ld_kp * delta_x;
    RCLCPP_DEBUG(get_logger(), "offset: %d", offset);

    int servo_value = 1500 + offset;

    servo_value = clamp(servo_value, this->servo_min, this->servo_max);

    return servo_value;
}

// ============== Vicon Tracking Functions ========================================

double Vehicle::vicon_get_x() { return this->x; }

double Vehicle::vicon_get_y() { return this->y; }

double Vehicle::vicon_get_bearing() { return this->bearing; }

float Vehicle::vicon_cal_dis(double target_x, double target_y) const {
    // Calculate distance using Pythagorean theorem
    return std::sqrt(std::pow(target_x - x, 2) + std::pow(target_y - y, 2));
}

void Vehicle::vicon_update_pose(pose_data pose) {
    // get position information from Vicon data
    this->x = pose.position.x * 1000.0;  // convert to mm
    this->y = pose.position.y * 1000.0;  // convert to mm

    // get orientation information (quaternion to Euler angles)
    double w = pose.orientation.w;
    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;

    // convert quaternion to bearing (degrees from North)
    this->bearing =
        atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * (180.0 / M_PI);
}

double Vehicle::vicon_cal_bearing(pair<double, double> target) const {
    double x2 = target.first;
    double y2 = target.second;

    float result = 0.0;
    std::vector<double> r = {x2 - x, y2 - y};
    std::vector<double> j = {0, 1};
    double mag_r = std::sqrt(std::inner_product(r.begin(), r.end(), r.begin(), 0.0));

    // Avoid division by zero
    if (mag_r < 0.001) {
        return bearing;  // Return current bearing if we're very close to target
    }

    double ry = std::inner_product(r.begin(), r.end(), j.begin(), 0.0);
    if (y > y2 && x > x2)
        result = -180.0 - (std::asin(ry / mag_r) * (180.0 / M_PI));
    else if (y < y2 && x > x2)
        result = 180.0 - (std::asin(ry / mag_r) * (180.0 / M_PI));
    else
        result = std::asin(ry / mag_r) * (180.0 / M_PI);

    return result;
}

float Vehicle::vicon_pro_controller(float error) {
    // Normalize the angle error to find the shortest rotation path (-180° to
    // +180°).
    // Without normalization, angles can wrap around causing inefficient turns.
    // Example: If target = -179° and current = 170°, raw error = -179 - 170 =
    // -349°
    //          This would cause a 349° CCW rotation (almost a full turn)
    //          But the shortest path is actually 11° CW: -179° to 180° to 170°
    //          = +11° After adding 360°: -349° + 360° = 11° (correct shortest
    //          path)
    //
    // This code tests three possibilities and chooses the one with smallest
    // absolute value:
    // 1. Original error (no wrapping)
    // 2. Error + 360° (wrap forward through +180°)
    float mag_error = fabs(error);
    if (fabs(error + 360) < mag_error) {
        error = error + 360;
    } else if (fabs(error - 360) < mag_error) {
        error = error - 360;
    }

    // calculate the offset from 1500 using the error (delta)
    float offset = this->vicon_kp * error;
    float servo_value = 1500 - offset;

    servo_value = clamp((int)servo_value, this->servo_min, this->servo_max);

    return servo_value;
}

// ============== Control Integration Functions ===================================

void Vehicle::control_step(pair<double, double> target_pos, int delta_x,
                           double threshold) {
    // calculate the vicon servo value in us
    double target_bearing = vicon_cal_bearing(target_pos);
    double error = target_bearing - bearing;
    int vicon_servo = vicon_pro_controller(error);

    // lane detection servo value in us
    int ld_servo = ld_pro_controller(delta_x);
    //  combine land detection confidence together
    double total_ld_conf = this->sym_conf;
    double ld_correction = (ld_servo - vicon_servo) * total_ld_conf;

    // the lane detection will not correct servo value for than 200 us
    ld_correction = clamp(ld_correction, -200.0, 200.0);

    int fused_servo = 0.0;
    if (total_ld_conf > threshold) {
        fused_servo = vicon_servo + ld_correction;
    } else {
        // if the lane detection total confidence is too low, then just use the
        // vicon servo.
        fused_servo = vicon_servo;
    }

    this->update_steering(fused_servo);
    this->update_speed();

    RCLCPP_DEBUG(this->get_logger(),
                 "{Target bearing: %f} {Current bearing: %f} {Error: %f} {Vicon Servo: "
                 "%d} \n"
                 "{total confi = %f =  %f } \n"
                 "{ld_servo: %d} {ld_correction %f} { Fused Servo : % d } ",
                 target_bearing, this->bearing, error, vicon_servo, total_ld_conf,
                 sym_conf, ld_servo, ld_correction, fused_servo);
}

// ============== Utility Functions ===============================================

void Vehicle::print() const {
    RCLCPP_INFO(get_logger(), "(%f, %f)<%f Degrees", this->x, this->y, this->bearing);
}
}  // namespace vehicle_ns