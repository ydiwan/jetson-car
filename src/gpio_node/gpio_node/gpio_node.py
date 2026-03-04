#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO



class GPIONode(Node):
    def __init__(self):
        super().__init__('gpio_node')
        
        # Pin definitions
        self.PWM_R = 15
        self.DIR_R = 31
        self.PWM_L = 32
        self.DIR_L = 7
        
        # PWM frequencies
        self.MOTOR_PWM_FREQ = 1000  # 1kHz for motors
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        
        # Setup pins
        GPIO.setup(self.PWM_R, GPIO.OUT)
        self.pwm_right = GPIO.PWM(self.PWM_R, self.MOTOR_PWM_FREQ)
        
        GPIO.setup(self.PWM_L, GPIO.OUT)
        self.pwm_left = GPIO.PWM(self.PWM_L, self.MOTOR_PWM_FREQ)
        
        GPIO.setup(self.DIR_R, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.DIR_L, GPIO.OUT, initial=GPIO.HIGH)
        
        # Start PWM with 0% duty cycle
        self.pwm_right.start(0)
        self.pwm_left.start(0)
        
        # Flag to track if PWMs are running
        self.pwms_running = True
        
        
        self.pwm_right_sub = self.create_subscription(
            Int32,
            'gpio/pwm_right',
            self.pwm_right_callback,
            qos_profile_sensor_data
        )
        
        self.pwm_left_sub = self.create_subscription(
            Int32,
            'gpio/pwm_left',
            self.pwm_left_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info('GPIO Node initialized')
        self.get_logger().info(f'Right motor PWM on pin {self.PWM_R}, DIR on pin {self.DIR_R}')
        self.get_logger().info(f'Left motor PWM on pin {self.PWM_L}, DIR on pin {self.DIR_L}')
    

    
    def pwm_right_callback(self, msg):
        duty = msg.data
        
        # Handle Direction
        if duty < 0:
            GPIO.output(self.DIR_R, GPIO.LOW)  # Reverse
            duty = abs(duty)
        else:
            GPIO.output(self.DIR_R, GPIO.HIGH) # Forward
            
        # Constrain and Calculate
        if duty > 1000:
            duty = 1000
            
        duty_percentage = (duty / 1000.0) * 100.0
        self.pwm_right.ChangeDutyCycle(duty_percentage)
        self.get_logger().debug(f'Right motor PWM: {msg.data} ({duty_percentage:.1f}%)')
    
    def pwm_left_callback(self, msg):
        duty = msg.data        
        
        # Handle Direction
        if duty < 0:
            GPIO.output(self.DIR_L, GPIO.LOW)  # Reverse
            duty = abs(duty)
        else:
            GPIO.output(self.DIR_L, GPIO.HIGH) # Forward
            
        # Constrain and Calculate
        if duty > 1000:
            duty = 1000
            
        duty_percentage = (duty / 1000.0) * 100.0
        self.pwm_left.ChangeDutyCycle(duty_percentage)
        self.get_logger().debug(f'Left motor PWM: {msg.data} ({duty_percentage:.1f}%)')
    
def destroy_node(self):
    """Cleanup GPIO on shutdown"""
    self.get_logger().info('Shutting down GPIO Node')
    
    if self.pwms_running:
        try:
            # Stop all PWM
            self.pwm_right.stop()
            self.pwm_left.stop()
            self.pwms_running = False
            
            # Set motor PWM pins to high impedance (INPUT)
            GPIO.setup(self.PWM_R, GPIO.IN)
            GPIO.setup(self.PWM_L, GPIO.IN)
            self.get_logger().info('Motor PWM pins set to high impedance')
            
        except Exception as e:
            self.get_logger().debug(f'Error during cleanup: {e}')
    
    
    super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    gpio_node = GPIONode()
    
    try:
        rclpy.spin(gpio_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        try:
            gpio_node.destroy_node()
        except Exception:
            pass
        
        # Only shutdown if context is valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()