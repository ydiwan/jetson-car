#ifndef VEHICLE_H
#define VEHICLE_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <utility>

#include "vehicle_controller/msc_if.h"

namespace vehicle_ns {
// ============== Adding Names =========================================================
// i hate ros naming
using pose_data = geometry_msgs::msg::PoseStamped::_pose_type;
using int_msg = std_msgs::msg::Int32;
using int_pub = rclcpp::Publisher<int_msg>::SharedPtr;
using std::pair;

class Vehicle {
   private:
    // ============== Private Member Variables ========================================
    double x;         ///< current x Position
    double y;         ///< current y Position
    double bearing;   ///< current bearing in degree from north
    double vicon_kp;  ///< proportional gain of Vicon proportional controller
    double ld_kp;     ///< proportional gain of lane detection proportional controller
    double alpha;     ///< complementary alpha value
    int servo_min;    ///< max servo plusewidth in micro-sec
    int servo_max;    ///< min servo plusewidth in micro-sec
    int servo;        ///< current servo pulse width in micro-sec
    int left_pwm;     ///< current left PWM
    int right_pwm;    ///< current right PWM
    double pos_conf;  ///< lane detection position confidence
    double sym_conf;  ///< lane detection symmetrical confidence

    // ============== ROS Publishers
    // ====================================================== Publish left pwm and right
    // pwm for the gpio node
    int_pub right_pwm_pub;
    int_pub left_pwm_pub;

    // ============== Hardware Interface
    // ================================================== The Jetson Orin Nano has only 8
    // bits of resolution for the PWM reg so we must use an extern servo controller.
    msc_if sc;

   public:
    // ============== Constructors/Destructors ========================================
    /**
     * @brief Constructs a Vehicle object with control parameters and hardware interfaces
     *
     * Initializes the vehicle with position tracking capabilities, motor control,
     * and servo steering. Sets up ROS publishers for PWM communication and
     * configures the servo controller interface.
     *
     * @param vicon_kp Proportional gain for Vicon-based position control (typical:
     * 0.1-2.0)
     * @param ld_kp Proportional gain for lane detection control (typical: 0.5-3.0)
     * @param alpha Complementary filter alpha value for sensor fusion (0.0-1.0)
     * @param servo_min Minimum servo pulse width in microseconds (typical: 1000)
     * @param servo_max Maximum servo pulse width in microseconds (typical: 2000)
     * @param servo_usb USB device path for servo controller (e.g., "/dev/ttyUSB0")
     * @param right_pwm_pub ROS publisher for right motor PWM values
     * @param left_pwm_pub ROS publisher for left motor PWM values
     *
     * @note Initial position is set to (0,0) with 0° bearing, servo centered at 1500μs,
     *       motors at half speed (500 PWM), and confidence values at 0.0
     */
    Vehicle(double vicon_kp, double ld_kp, double alpha, int servo_min, int servo_max,
            std::string servo_usb, int_pub right_pwm_pub,
            int_pub left_pwm_pub)
        : x(0.0),                        // default value is 0
          y(0.0),                        // default value is 0
          bearing(0.0),                  // default value is 0
          vicon_kp(vicon_kp),            // assign to the arg
          ld_kp(ld_kp),                  // assign to the arg
          alpha(alpha),                  // assigned to the arg
          servo_min(servo_min),          // assign to the arg
          servo_max(servo_max),          // assign to the arg
          servo(1500),                   // Set the servo center
          left_pwm(500),                 // set the motor to half speed
          right_pwm(500),                // set the motor to half speed
          pos_conf(0.0),                 // assume the confidence is Zero
          sym_conf(0.0),                 // assume the confidence is Zero
          right_pwm_pub(right_pwm_pub),  // Initialize publishers
          left_pwm_pub(left_pwm_pub),    // Initialize publishers
          sc(servo_usb.c_str())          // pass the usb path arg to msc_if constructor
          {};
    
          ~Vehicle(){
            motor_on(false);
          }
    // ============== Actuators Control Functions =====================================
    void motor_on(bool on);
    
    
    /**
     * @brief Automatically calculates and sets differential motor speeds based on
     * steering
     *
     * Implements differential steering by slowing the inner wheel during turns.
     * The servo position determines turn coefficient: at servo_max, right motor
     * is slowest; at servo_min, left motor is slowest. This enables tighter turns
     * by creating speed differential between wheels.
     *
     * Speed calculation:
     * - turn_coeff = (servo - servo_min) / servo_range
     * - left_pwm = 1000 * (1.0 - turn_coeff)
     * - right_pwm = 1000 * turn_coeff
     *
     * @note PWM values are clamped to safe range [250, 950] and published to ROS topics
     * @see update_speed(int, int) for manual PWM control
     */
    void update_speed();

    /**
     * @brief Directly sets left and right motor PWM values
     *
     * Bypasses automatic differential steering calculation and directly
     * controls motor speeds. Useful for manual control, testing, or
     * specialized maneuvers not handled by differential steering.
     *
     * @param l_pwm Left motor PWM value (0-1000, clamped automatically)
     * @param r_pwm Right motor PWM value (0-1000, clamped automatically)
     *
     * @note Values are published to ROS topics for GPIO node consumption
     * @see update_speed() for automatic differential steering
     */
    void update_speed(int l_pwm, int r_pwm);

    /**
     * @brief Sets servo steering angle via external servo controller
     *
     * Updates the steering servo position using the external servo controller
     * interface. The servo value is automatically clamped to configured
     * min/max limits to prevent mechanical damage.
     *
     * @param servo_value Servo pulse width in microseconds (typically 1000-2000)
     *                   1500μs = center, <1500 = left turn, >1500 = right turn
     *
     * @note Uses external servo controller due to Jetson Orin Nano's limited PWM
     * resolution
     */
    void update_steering(int servo_value);

    // ============== Lane Detection Functions ========================================
    /**
     * @brief Updates the position confidence score from lane detection
     *
     * Sets confidence in lane detection's ability to accurately determine
     * the vehicle's position relative to lane center. Higher confidence
     * indicates more reliable lane detection output.
     *
     * @param pos_conf Position confidence value (0.0-1.0)
     *                0.0 = no confidence, 1.0 = maximum confidence
     *
     * @note Used in sensor fusion to weight lane detection vs Vicon control
     */
    void set_pos_conf(double pos_conf);

    /**
     * @brief Updates the symmetry confidence score from lane detection
     *
     * Sets confidence in lane detection's assessment of lane symmetry.
     * Higher values indicate both lane lines are clearly visible and
     * symmetrically positioned, improving steering accuracy.
     *
     * @param sym_conf Symmetry confidence value (0.0-1.0)
     *                0.0 = asymmetric/poor detection, 1.0 = perfect symmetry
     *
     * @note Weighted more heavily (0.6) than position confidence (0.4) in fusion
     */
    void set_sym_conf(double sym_conf);

    /**
     * @brief Lane detection proportional controller for lane centering
     *
     * Calculates servo correction to center the vehicle between lane lines.
     * Uses proportional control with configured gain to convert pixel offset
     * into servo microsecond adjustment.
     *
     * @param delta_x Horizontal pixel offset from lane center
     *               Positive = vehicle right of center, Negative = vehicle left of
     * center
     *
     * @return Servo pulse width in microseconds (clamped to servo_min/servo_max)
     *
     * @note Control equation: servo_value = 1500 + (ld_kp * delta_x)
     *       1500μs represents centered steering position
     */
    int ld_pro_controller(int delta_x);

    // ============== Vicon Tracking Functions ========================================
    /**
     * @brief Gets current vehicle X position from Vicon tracking
     *
     * @return X coordinate in millimeters from Vicon coordinate system origin
     */
    double vicon_get_x();

    /**
     * @brief Gets current vehicle Y position from Vicon tracking
     *
     * @return Y coordinate in millimeters from Vicon coordinate system origin
     */
    double vicon_get_y();

    /**
     * @brief Gets current vehicle bearing from Vicon tracking
     *
     * @return Bearing angle in degrees from North (0°=North, 90°=East, etc.)
     */
    double vicon_get_bearing();

    /**
     * @brief Updates vehicle pose from Vicon tracking system data
     *
     * Processes ROS PoseStamped message from Vicon tracking node, extracting
     * position (converted from meters to millimeters) and orientation
     * (converted from quaternion to bearing angle in degrees from North).
     *
     * @param pose ROS PoseStamped pose data from Vicon tracking system
     *            Contains position (x,y,z in meters) and orientation (quaternion)
     *
     * @note Position is converted to mm for consistency with other measurements
     *       Quaternion is converted to bearing using atan2 transformation
     */
    void vicon_update_pose(pose_data pose);

    /**
     * @brief Calculates straight-line distance to target position
     *
     * Uses Pythagorean theorem to compute Euclidean distance between
     * current vehicle position and specified target coordinates.
     *
     * @param x2 Target X coordinate in millimeters
     * @param y2 Target Y coordinate in millimeters
     *
     * @return Distance to target in millimeters
     *
     * @note Does not account for obstacles or path planning
     */
    float vicon_cal_dis(double x2, double y2) const;

    /**
     * @brief Calculates required bearing to reach target position
     *
     * Determines the optimal bearing angle (degrees from North) needed
     * to navigate directly to the target position. Handles all quadrants
     * and edge cases to ensure correct directional calculation.
     *
     * @param target Target position as std::pair<double, double>
     *              First element: target X coordinate (mm)
     *              Second element: target Y coordinate (mm)
     *
     * @return Required bearing in degrees from North (-180° to +180°)
     *        0° = North, 90° = East, -90° = West, ±180° = South
     *
     * @note Returns current bearing if very close to target (distance < 0.001mm)
     *       Uses vector mathematics with dot product for accurate calculation
     */
    double vicon_cal_bearing(pair<double, double> target) const;

    /**
     * @brief Vicon-based proportional controller for bearing correction
     *
     * Implements proportional control to minimize bearing error between current
     * and desired headings. Automatically handles angle wrapping to find the
     * shortest rotation path, preventing inefficient 300°+ turns when a small
     * opposite rotation would suffice.
     *
     * @param error Bearing error in degrees (target_bearing - current_bearing)
     *             Positive = need to turn clockwise, Negative = counterclockwise
     *
     * @return Servo pulse width in microseconds (clamped to servo limits)
     *        Values < 1500 = left turn, > 1500 = right turn
     *
     * @note Angle normalization tests three possibilities (error, error±360°)
     *       and chooses the smallest absolute value for optimal control
     *       Control equation: servo = 1500 - (vicon_kp * normalized_error)
     */
    float vicon_pro_controller(float error);

    // ============== Control Integration Functions ===================================
    /**
     * @brief Executes one control cycle with sensor fusion and actuation
     *
     * Performs complete vehicle control by fusing Vicon tracking and lane detection
     * inputs. Calculates steering corrections from both systems, weighs them by
     * confidence metrics, and applies the fused result to vehicle actuators.
     *
     * Control fusion logic:
     * 1. Calculate Vicon servo correction using bearing error
     * 2. Calculate lane detection servo correction using pixel offset
     * 3. Combine confidences: total_conf = 0.4*pos_conf + 0.6*sym_conf
     * 4. Apply lane correction only if total_conf > threshold
     * 5. Limit lane correction to ±200μs to prevent overcorrection
     * 6. Update steering and speed actuators
     *
     * @param target Target position as std::pair<double, double>
     *              (target_x_mm, target_y_mm) in Vicon coordinate system
     * @param delta Horizontal pixel offset from lane center
     *             Positive = right of center, Negative = left of center
     * @param threshold Minimum total confidence required for lane correction
     *                 Default: 0.7 (70% confidence threshold)
     *
     * @note If lane confidence < threshold, uses pure Vicon control
     *       Symmetry confidence weighted higher (60%) than position (40%)
     *       Automatically calls update_steering() and update_speed()
     *
     * @see set_pos_conf(), set_sym_conf() for confidence updates
     */
    void control_step(pair<double, double> target, int delta, double threshold = 0.7);

    // ============== Utility Functions ===============================================
    /**
     * @brief Prints current vehicle state to ROS logger
     *
     * Outputs current position coordinates and bearing to ROS INFO log.
     * Useful for debugging and monitoring vehicle state during operation.
     *
     * Format: "(x, y)<bearing Degrees"
     * Example: "(1250.5, -340.2)<45.7 Degrees"
     *
     * @note Position in millimeters, bearing in degrees from North
     */
    void print() const;

    /**
     * @brief Gets ROS logger instance for this class
     *
     * Provides centralized logging interface for all Vehicle class methods.
     * Uses static initialization to ensure single logger instance.
     *
     * @return Reference to ROS2 logger configured for "vehicle_class" namespace
     *
     * @note Static method - can be called without Vehicle instance
     */
    static const rclcpp::Logger &get_logger() {
        static rclcpp::Logger logger = rclcpp::get_logger("vehicle_class");
        return logger;
    }
};
}  // namespace vehicle_ns
#endif  // VEHICLE_H