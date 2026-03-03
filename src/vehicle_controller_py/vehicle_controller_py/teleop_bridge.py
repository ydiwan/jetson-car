import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from vehicle_controller_py.msc_if import MscIf

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')

        # 1. Setup Motor Publishers (Talking to your gpio_node)
        self.left_pub = self.create_publisher(Int32, '/gpio/pwm_left', 10)
        self.right_pub = self.create_publisher(Int32, '/gpio/pwm_right', 10)

        # 2. Setup Steering (Talking directly to Maestro via USB)
        try:
            self.servo_controller = MscIf('/dev/ttyACM0')
            self.get_logger().info("Connected to Micro Maestro.")
            
            # If your motor driver requires the Maestro "Enable" pins to be pulled high:
            # self.servo_controller.set_target(1, 8000) 
            # self.servo_controller.set_target(2, 8000)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Maestro: {e}")

        # 3. Listen to the keyboard commands
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.twist_cb, 10)
        self.get_logger().info("Teleop Bridge Ready! Drive with W/A/S/D in the teleop terminal.")

    def twist_cb(self, msg):
        # --- STEERING ---
        # msg.angular.z is positive for Left, negative for Right (-1.0 to 1.0)
        # Map this to the Maestro Servo range (1000 to 2000, center 1500)
        steer = 1500 + int(msg.angular.z * 500)
        steer = max(1000, min(2000, steer)) # Clamp for safety
        self.servo_controller.set_servo(0, steer) 

        # --- SPEED ---
        # msg.linear.x is Forward/Backward (0.0 to 1.0)
        # Map this to your PWM range (0 to 900)
        speed = int(abs(msg.linear.x) * 1500)
        speed = max(0, min(900, speed)) # Clamp for safety

        # Send identical power to both back wheels (Standard RC control)
        pwm_msg = Int32()
        pwm_msg.data = speed
        self.left_pub.publish(pwm_msg)
        self.right_pub.publish(pwm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()