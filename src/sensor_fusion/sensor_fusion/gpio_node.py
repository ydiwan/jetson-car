import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO

class GPIONode(Node):
    def __init__(self):
        super().__init__('gpio_node')
        
        self.PWM_R = 15
        self.DIR_R = 31
        self.PWM_L = 32
        self.DIR_L = 7
        self.MOTOR_PWM_FREQ = 1000
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.PWM_R, GPIO.OUT)
        self.pwm_right = GPIO.PWM(self.PWM_R, self.MOTOR_PWM_FREQ)
        GPIO.setup(self.PWM_L, GPIO.OUT)
        self.pwm_left = GPIO.PWM(self.PWM_L, self.MOTOR_PWM_FREQ)
        
        GPIO.setup(self.DIR_R, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.DIR_L, GPIO.OUT, initial=GPIO.HIGH)
        
        # Active-low
        self.pwm_right.start(100.0)
        self.pwm_left.start(100.0)
        self.pwms_running = True
        
        # Subscriptions
        self.pwm_right_sub = self.create_subscription(Int32, 'gpio/pwm_right', self.pwm_right_callback, 10)
        self.pwm_left_sub = self.create_subscription(Int32, 'gpio/pwm_left', self.pwm_left_callback, 10)
        
        self.get_logger().info('GPIO Node initialized. Listening for C++ Bridge...')

    def pwm_right_callback(self, msg):
        duty = msg.data
        if duty < 0:
            GPIO.output(self.DIR_R, GPIO.LOW)
            duty = abs(duty)
        else:
            GPIO.output(self.DIR_R, GPIO.HIGH)
            
        duty_percentage = (min(duty, 1000) / 1000.0) * 100.0
        self.pwm_right.ChangeDutyCycle(duty_percentage)
        self.get_logger().debug(f'R_Motor Command Received: {duty_percentage:.1f}%') 
    
    def pwm_left_callback(self, msg):
        duty = msg.data        
        if duty < 0:
            GPIO.output(self.DIR_L, GPIO.LOW)
            duty = abs(duty)
        else:
            GPIO.output(self.DIR_L, GPIO.HIGH)
            
        duty_percentage = (min(duty, 1000) / 1000.0) * 100.0
        self.pwm_left.ChangeDutyCycle(duty_percentage)
        self.get_logger().debug(f'L_Motor Command Received: {duty_percentage:.1f}%')
    
    def destroy_node(self):
        self.get_logger().info('Shutting down GPIO Node')
        if self.pwms_running:
            try:
                self.pwm_right.ChangeDutyCycle(100.0)
                self.pwm_left.ChangeDutyCycle(100.0)
                self.pwm_right.stop()
                self.pwm_left.stop()
                self.pwms_running = False
            except Exception as e:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gpio_node = GPIONode()
    try:
        rclpy.spin(gpio_node)
    except KeyboardInterrupt:
        pass
    finally:
        gpio_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()