import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from vehicle_controller_py.msc_if import MscIf

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')

        # Setup Motor Publishers
        self.left_pub = self.create_publisher(Int32, '/gpio/pwm_left', 10)
        self.right_pub = self.create_publisher(Int32, '/gpio/pwm_right', 10)

        # 2. Setup Steering (Talking directly to Maestro via USB)
        try:
            self.servo_controller = MscIf('/dev/ttyACM0')
            self.get_logger().info("Connected to Micro Maestro.")
            
            # Set the Maestro 'Enable' pins to HIGH
            self.servo_controller.set_servo(1, 8000) 
            self.servo_controller.set_servo(2, 8000)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Maestro: {e}")

        # Listen to the keyboard commands
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.twist_cb, 10)
        self.get_logger().info("Teleop Bridge Ready! Drive with W/A/S/D in the teleop terminal.")

    def twist_cb(self, msg):
        # STEERING (j=Left, l=Right)
        # msg.angular.z: Positive is Left, Negative is Right. 
        # Subtract the offset from 1500 to flip the direction.
        steer = 1500 - int(msg.angular.z * 500)
        steer = max(1000, min(2000, steer)) 
        self.servo_controller.set_servo(0, steer) 

        # SPEED (Always Positive / Forward Only)
        base_speed = int(abs(msg.linear.x) * 900) 

        # SMART ENABLE/DISABLE
        # Only kill the motors if BOTH linear movement and turning are zero.
        if base_speed == 0 and abs(msg.angular.z) < 0.05:
            left_speed = 0
            right_speed = 0
            self.servo_controller.set_servo(1, 0) # Motor OFF
            self.servo_controller.set_servo(2, 0)
        else:
            self.servo_controller.set_servo(1, 8000) # Motor ON
            self.servo_controller.set_servo(2, 8000)
            
            # Differential Math for smoother turns
            differential_strength = 0.30 
            offset = int(base_speed * differential_strength * msg.angular.z)
            
            left_speed = base_speed - offset
            right_speed = base_speed + offset

        # Final Clamp
        left_speed = max(0, min(900, left_speed))
        right_speed = max(0, min(900, right_speed))

        self.get_logger().info(f"Steer: {steer} | L: {left_speed} R: {right_speed}")

        # Publish to GPIO Node
        left_msg = Int32()
        left_msg.data = left_speed
        self.left_pub.publish(left_msg)

        right_msg = Int32()
        right_msg.data = right_speed
        self.right_pub.publish(right_msg)

        # Clamp both speeds to prevent sending invalid PWMs to the hardware
        left_speed = max(-900, min(900, left_speed))
        right_speed = max(-900, min(900, right_speed))
        
        self.get_logger().info(f"Sending PWM -> Left: {left_speed}, Right: {right_speed}")

        # Publish the independent speeds to the Jetson GPIO node
        left_msg = Int32()
        left_msg.data = left_speed
        self.left_pub.publish(left_msg)

        right_msg = Int32()
        right_msg.data = right_speed
        self.right_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()