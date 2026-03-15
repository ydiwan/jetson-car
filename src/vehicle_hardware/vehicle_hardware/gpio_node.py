import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import qos_profile_sensor_data
import Jetson.GPIO as GPIO

class JetsonGPIONode(Node):
    def __init__(self):
        super().__init__('gpio_node')

        # Pin Configuration 
        self.R_PWM_PIN = 15
        self.R_DIR_PIN = 31
        self.L_PWM_PIN = 32
        self.L_DIR_PIN = 7
        self.PWM_FREQ = 1000

        # Setup GPIO 
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([self.R_DIR_PIN, self.L_DIR_PIN], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup([self.R_PWM_PIN, self.L_PWM_PIN], GPIO.OUT, initial=GPIO.LOW) # LOW = 0V

        self.r_pwm = GPIO.PWM(self.R_PWM_PIN, self.PWM_FREQ)
        self.l_pwm = GPIO.PWM(self.L_PWM_PIN, self.PWM_FREQ)

        # Start at 0% Duty Cycle (Perfectly Stopped)
        self.r_pwm.start(0.0)
        self.l_pwm.start(0.0)

        # Subscriptions
        self.l_sub = self.create_subscription(Int32, 'gpio/pwm_left', self.left_callback, qos_profile_sensor_data)
        self.r_sub = self.create_subscription(Int32, 'gpio/pwm_right', self.right_callback, qos_profile_sensor_data)

        self.get_logger().info("GPIO Node Active: Standard Active-High Logic.")

    def set_motor(self, pwm_obj, dir_pin, target_velocity):
        # Set Direction
        if target_velocity >= 0:
            GPIO.output(dir_pin, GPIO.HIGH) # Forward
        else:
            GPIO.output(dir_pin, GPIO.LOW)  # Reverse

        # Calculate Standard Duty Cycle (0.0 to 100.0)
        speed_percent = abs(target_velocity) / 10.0

        # Clamp just to be safe
        speed_percent = max(min(speed_percent, 100.0), 0.0)

        # Apply
        pwm_obj.ChangeDutyCycle(speed_percent)

    def left_callback(self, msg: Int32):
        self.set_motor(self.l_pwm, self.L_DIR_PIN, msg.data)

    def right_callback(self, msg: Int32):
        self.set_motor(self.r_pwm, self.R_DIR_PIN, msg.data)

    def destroy_node(self):
        self.r_pwm.stop()
        self.l_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonGPIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()