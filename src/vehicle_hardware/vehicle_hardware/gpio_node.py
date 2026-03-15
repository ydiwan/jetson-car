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
        
        # Start as inputs, internal pull-up turns motors off
        GPIO.setup(self.R_PWM_PIN, GPIO.IN)
        GPIO.setup(self.L_PWM_PIN, GPIO.IN)

        self.r_pwm = None
        self.l_pwm = None

        # Subscriptions
        self.l_sub = self.create_subscription(Int32, 'gpio/pwm_left', self.left_callback, qos_profile_sensor_data)
        self.r_sub = self.create_subscription(Int32, 'gpio/pwm_right', self.right_callback, qos_profile_sensor_data)

        self.get_logger().info("GPIO Node Active: Using High-Z Trick for perfect braking.")

    def set_motor(self, is_left, target_velocity):
        pwm_pin = self.L_PWM_PIN if is_left else self.R_PWM_PIN
        dir_pin = self.L_DIR_PIN if is_left else self.R_DIR_PIN
        
        if target_velocity == 0:
            if is_left and self.l_pwm:
                self.l_pwm.stop()
                self.l_pwm = None
                GPIO.cleanup(pwm_pin) # Force release the pin
            elif not is_left and self.r_pwm:
                self.r_pwm.stop()
                self.r_pwm = None
                GPIO.cleanup(pwm_pin) # Force release the pin
                
            # Turns motors off
            GPIO.setup(pwm_pin, GPIO.IN)
            return

        # Turn pin back into an OUTPUT
        GPIO.setup(pwm_pin, GPIO.OUT)
        
        # Set Direction
        if target_velocity >= 0:
            GPIO.output(dir_pin, GPIO.HIGH) # Forward
        else:
            GPIO.output(dir_pin, GPIO.LOW)  # Reverse

        # Calculate Active-Low Duty Cycle
        speed_percent = abs(target_velocity) / 10.0
        active_low_duty = 100.0 - speed_percent

        # Start or Update PWM
        if is_left:
            if not self.l_pwm:
                self.l_pwm = GPIO.PWM(pwm_pin, self.PWM_FREQ)
                self.l_pwm.start(active_low_duty)
            else:
                self.l_pwm.ChangeDutyCycle(active_low_duty)
        else:
            if not self.r_pwm:
                self.r_pwm = GPIO.PWM(pwm_pin, self.PWM_FREQ)
                self.r_pwm.start(active_low_duty)
            else:
                self.r_pwm.ChangeDutyCycle(active_low_duty)

    def left_callback(self, msg: Int32):
        self.set_motor(True, msg.data)

    def right_callback(self, msg: Int32):
        self.set_motor(False, msg.data)

    def destroy_node(self):
        if self.l_pwm: self.l_pwm.stop()
        if self.r_pwm: self.r_pwm.stop()
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