import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
import math

class AckermannKinematicsNode(Node):
    def __init__(self):
        super().__init__('ackermann_kinematics_node')
        
        # Declare and Fetch Parameters
        self.declare_parameter('wheelbase', 0.148)   
        self.declare_parameter('track_width', 0.133) 
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('max_motor_speed', 16.65) # 159 RPM max
        
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.R_wheel = self.get_parameter('wheel_radius').value
        self.max_rad_s = self.get_parameter('max_motor_speed').value

        # Setup Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Setup Publishers
        self.steering_pub = self.create_publisher(Float32, '/vehicle/steering_angle', 10)
        
        # Publish to the Jetson GPIO node instead of Maestro
        self.l_pwm_pub = self.create_publisher(Int32, 'gpio/pwm_left', qos_profile_sensor_data)
        self.r_pwm_pub = self.create_publisher(Int32, 'gpio/pwm_right', qos_profile_sensor_data)
        
        self.get_logger().info(
            f"Ackermann Kinematics initialized with L={self.L}m, W={self.W}m, R={self.R_wheel}m"
        )

    def cmd_vel_callback(self, msg: Twist):
        v_x = msg.linear.x       
        omega_z = msg.angular.z  

        if v_x == 0.0:
            steering_angle = 0.0
            rad_s_left = 0.0
            rad_s_right = 0.0
        else:
            steering_angle = math.atan((self.L * omega_z) / v_x)
            v_left = v_x - (omega_z * self.W / 2.0)
            v_right = v_x + (omega_z * self.W / 2.0)
            rad_s_left = v_left / self.R_wheel
            rad_s_right = v_right / self.R_wheel

        # Publish Steering
        steer_msg = Float32()
        steer_msg.data = steering_angle
        self.steering_pub.publish(steer_msg)

        # Map rad/s (-16.65 to 16.65) to Duty Cycle range (-1000 to 1000)
        l_duty = int((rad_s_left / self.max_rad_s) * 1000)
        r_duty = int((rad_s_right / self.max_rad_s) * 1000)

        # Clamp values to safe ranges
        l_duty = max(min(l_duty, 1000), -1000)
        r_duty = max(min(r_duty, 1000), -1000)

        self.l_pwm_pub.publish(Int32(data=l_duty))
        self.r_pwm_pub.publish(Int32(data=r_duty))

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