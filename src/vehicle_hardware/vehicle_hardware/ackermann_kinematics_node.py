import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
import math

class AckermannKinematicsNode(Node):
    def __init__(self):
        super().__init__('ackermann_kinematics_node')
        
        # Declare and Fetch Parameters 
        self.declare_parameter('wheelbase', 0.148)   
        self.declare_parameter('track_width', 0.133) 
        self.declare_parameter('wheel_radius', 0.034)
        
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.R_wheel = self.get_parameter('wheel_radius').value

        # Setup Subscriptions
        # Listens for target velocity commands (linear and angular)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Setup Publishers
        # Publishes target steering angle in radians for the front servo
        self.steering_pub = self.create_publisher(Float32, '/vehicle/steering_angle', 10)
        
        # Publishes target rotational speed in rad/s for [left_motor, right_motor]
        self.wheel_speed_pub = self.create_publisher(Float32MultiArray, '/vehicle/wheel_speeds', 10)
        
        self.get_logger().info(
            f"Ackermann Kinematics initialized with L={self.L}m, W={self.W}m, R={self.R_wheel}m"
        )

    def cmd_vel_callback(self, msg: Twist):
        v_x = msg.linear.x       # Target forward speed (m/s)
        omega_z = msg.angular.z  # Target rotational speed of the whole car (rad/s)

        # Handle the case where the car is stopped or trying to spin in place
        if v_x == 0.0:
            steering_angle = 0.0
            rad_s_left = 0.0
            rad_s_right = 0.0
        else:
            # Calculate front steering angle: delta = arctan(L * omega_z / v_x)
            steering_angle = math.atan((self.L * omega_z) / v_x)
            
            # Calculate linear speeds for the left and right rear wheels 
            # The outside wheel must travel faster than the inside wheel during a turn
            v_left = v_x - (omega_z * self.W / 2.0)
            v_right = v_x + (omega_z * self.W / 2.0)

            # Convert linear wheel speeds (m/s) to rotational wheel speeds (rad/s)
            # omega_wheel = v / R_wheel
            rad_s_left = v_left / self.R_wheel
            rad_s_right = v_right / self.R_wheel
   
        # Publish Steering
        steer_msg = Float32()
        steer_msg.data = steering_angle
        self.steering_pub.publish(steer_msg)

        # Publish Wheel Speeds [Left, Right]
        wheel_msg = Float32MultiArray()
        wheel_msg.data = [float(rad_s_left), float(rad_s_right)]
        self.wheel_speed_pub.publish(wheel_msg)
        
        # Debug output
        self.get_logger().debug(
            f"Cmd(v={v_x:.2f}, w={omega_z:.2f}) -> "
            f"Steer={math.degrees(steering_angle):.1f}deg | "
            f"Wheels(rad/s)=[L:{rad_s_left:.2f}, R:{rad_s_right:.2f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AckermannKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()