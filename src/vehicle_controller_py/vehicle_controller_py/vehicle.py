import math
import rclpy
from std_msgs.msg import Int32
from .msc_if import MscIf  # Import the Micro Maestro interface we wrote

class Vehicle:
    def __init__(self, vicon_kp: float, ld_kp: float, alpha: float, 
                 servo_min: int, servo_max: int, servo_usb: str, 
                 right_pwm_pub, left_pwm_pub):
        
        self.logger = rclpy.logging.get_logger("vehicle_class")
        
        # ============== Private Member Variables ========================================
        self.x = 0.0               # current x Position
        self.y = 0.0               # current y Position
        self.bearing = 0.0         # current bearing in degree from north
        self.vicon_kp = vicon_kp   # proportional gain of Vicon
        self.ld_kp = ld_kp         # proportional gain of lane detection
        self.alpha = alpha         # complementary alpha value
        self.servo_min = servo_min # min servo pulsewidth in micro-sec
        self.servo_max = servo_max # max servo pulsewidth in micro-sec
        
        self.servo = 1500          # current servo pulse width (center)
        self.left_pwm = 500        # current left PWM (half speed)
        self.right_pwm = 500       # current right PWM (half speed)
        
        self.pos_conf = 0.0        # position confidence
        self.sym_conf = 0.0        # symmetrical confidence
        
        # ============== ROS Publishers ==================================================
        self.right_pwm_pub = right_pwm_pub
        self.left_pwm_pub = left_pwm_pub
        
        # ============== Hardware Interface ==============================================
        self.sc = MscIf(servo_usb, logger=self.logger)

    def __del__(self):
        self.motor_on(False)

    # ============== Actuators Control Functions =====================================
    def motor_on(self, on: bool):
        if on:
            self.sc.set_pin(1, 1)
            self.sc.set_pin(2, 1)
        else:
            self.sc.set_pin(1, 0)
            self.sc.set_pin(2, 0)

    def update_speed_auto(self):
        """Automatically calculates differential motor speeds based on steering."""
        servo_range = self.servo_max - self.servo_min
        if servo_range == 0:
            turn_coeff = 0.5
        else:
            turn_coeff = (self.servo - self.servo_min) / float(servo_range)

        left_pwm = int(1000 * (1.0 - turn_coeff))
        right_pwm = int(1000 * turn_coeff)

        # Ensure speed is within valid range (clamped to 900 as in C++)
        self.left_pwm = max(0, min(900, left_pwm))
        self.right_pwm = max(0, min(900, right_pwm))

        self.logger.debug(
            f"Auto motor pwm: servo_range={servo_range}, turn_coeff={turn_coeff:.3f}, "
            f"left={self.left_pwm}, right={self.right_pwm}"
        )

        self._publish_pwms()

    def update_speed_manual(self, l_pwm: int, r_pwm: int):
        """Directly sets left and right motor PWM values."""
        self.left_pwm = max(0, min(1000, l_pwm))
        self.right_pwm = max(0, min(1000, r_pwm))

        self.logger.debug(f"Manual pwm: left={self.left_pwm}, right={self.right_pwm}")
        self._publish_pwms()

    def _publish_pwms(self):
        left_msg = Int32(data=self.left_pwm)
        right_msg = Int32(data=self.right_pwm)
        self.left_pwm_pub.publish(left_msg)
        self.right_pwm_pub.publish(right_msg)

    def update_steering(self, servo_value: int):
        self.servo = max(self.servo_min, min(self.servo_max, servo_value))
        self.sc.set_servo(0, self.servo)

    # ============== Lane Detection Functions ========================================
    def set_pos_conf(self, pos_conf: float):
        self.pos_conf = pos_conf

    def set_sym_conf(self, sym_conf: float):
        self.sym_conf = sym_conf

    def ld_pro_controller(self, delta_x: int) -> int:
        offset = int(self.ld_kp * delta_x)
        self.logger.debug(f"offset: {offset}")
        servo_value = 1500 + offset
        return max(self.servo_min, min(self.servo_max, servo_value))

    # ============== Vicon Tracking Functions ========================================
    def vicon_get_x(self) -> float: return self.x
    def vicon_get_y(self) -> float: return self.y
    def vicon_get_bearing(self) -> float: return self.bearing

    def vicon_cal_dis(self, target_x: float, target_y: float) -> float:
        return math.hypot(target_x - self.x, target_y - self.y)

    def vicon_update_pose(self, pose):
        self.x = pose.position.x * 1000.0  # mm
        self.y = pose.position.y * 1000.0  # mm

        w, x, y, z = pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
        
        # Convert quaternion to bearing (degrees from North)
        self.bearing = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def vicon_cal_bearing(self, target: tuple) -> float:
        x2, y2 = target
        r_x = x2 - self.x
        r_y = y2 - self.y
        mag_r = math.hypot(r_x, r_y)

        if mag_r < 0.001:
            return self.bearing

        val = math.degrees(math.asin(r_y / mag_r))
        
        if self.y > y2 and self.x > x2:
            return -180.0 - val
        elif self.y < y2 and self.x > x2:
            return 180.0 - val
        else:
            return val

    def vicon_pro_controller(self, error: float) -> int:
        # Normalize the angle error
        mag_error = abs(error)
        if abs(error + 360) < mag_error:
            error += 360
        elif abs(error - 360) < mag_error:
            error -= 360

        offset = self.vicon_kp * error
        servo_value = int(1500 - offset)
        return max(self.servo_min, min(self.servo_max, servo_value))

    # ============== Control Integration Functions ===================================
    def control_step(self, target_pos: tuple, delta_x: int, threshold: float = 0.7):
        target_bearing = self.vicon_cal_bearing(target_pos)
        error = target_bearing - self.bearing
        vicon_servo = self.vicon_pro_controller(error)

        ld_servo = self.ld_pro_controller(delta_x)
        total_ld_conf = self.sym_conf
        
        ld_correction = (ld_servo - vicon_servo) * total_ld_conf
        ld_correction = max(-200.0, min(200.0, ld_correction))

        if total_ld_conf > threshold:
            fused_servo = int(vicon_servo + ld_correction)
        else:
            fused_servo = vicon_servo

        self.update_steering(fused_servo)
        self.update_speed_auto()

        self.logger.debug(
            f"{{Target: {target_bearing:.2f}}} {{Current: {self.bearing:.2f}}} {{Err: {error:.2f}}} {{Vicon: {vicon_servo}}}\n"
            f"{{total_conf={total_ld_conf:.3f} = {self.sym_conf:.3f}}}\n"
            f"{{ld_servo: {ld_servo}}} {{ld_corr: {ld_correction:.1f}}} {{Fused: {fused_servo}}}"
        )

    def print_state(self):
        self.logger.info(f"({self.x:.2f}, {self.y:.2f})<{self.bearing:.2f} Degrees")