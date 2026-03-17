import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
from rclpy.qos import qos_profile_sensor_data
import math

class AckermannKinematicsNode(Node):
    def __init__(self):
        super().__init__('ackermann_kinematics_node')
        
        # Parameters
        self.declare_parameter('wheelbase', 0.148)   
        self.declare_parameter('max_steer_angle', 0.52) # Needed for the turn_coeff mapping
        
        self.L = self.get_parameter('wheelbase').value
        self.max_angle = self.get_parameter('max_steer_angle').value

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.steering_pub = self.create_publisher(Float32, '/vehicle/steering_angle', 10)
        self.l_pwm_pub = self.create_publisher(Int32, 'gpio/pwm_left', qos_profile_sensor_data)
        self.r_pwm_pub = self.create_publisher(Int32, 'gpio/pwm_right', qos_profile_sensor_data)
        
        self.get_logger().info("Ackermann Node initialized with Inverted Electronic Differential.")

    def cmd_vel_callback(self, msg: Twist):
        v_x = msg.linear.x       
        omega_z = msg.angular.z  

        # Handle Dead Stop
        if v_x == 0.0:
            self.steering_pub.publish(Float32(data=0.0))
            # 1000 is the mathematical STOP state for Active-Low motors
            self.l_pwm_pub.publish(Int32(data=1000)) 
            self.r_pwm_pub.publish(Int32(data=1000)) 
            return

        # Calculate Physical Steering Angle
        steering_angle = math.atan((self.L * omega_z) / v_x)
        steering_angle = max(-self.max_angle, min(self.max_angle, steering_angle))
        
        steer_msg = Float32()
        steer_msg.data = steering_angle
        self.steering_pub.publish(steer_msg)
        
        """
        Map steering angle to turn coefficient (0.0 to 1.0)
        Left (+ angle) -> turn_coeff = 0.0
        Center (0.0) -> turn_coeff = 0.5
        Right (- angle) -> turn_coeff = 1.0
        """
        turn_coeff = (-steering_angle + self.max_angle) / (2.0 * self.max_angle)
        turn_coeff = max(0.0, min(1.0, turn_coeff))

        # Base Effort
        # Normalize the keyboard speed (0.73) into a 0.0 to 1.0 percentage
        effort = abs(v_x)
        effort = max(0.0, min(1.0, effort))

        # Calculate Speed Percentages 
        # When turning left (coeff=0), inside wheel speed is 0, outside is max
        left_speed_pct = effort * turn_coeff       
        right_speed_pct = effort * (1.0 - turn_coeff)

        # Invert since motors are active-low
        left_pwm = int(1000 - (left_speed_pct * 1000))
        right_pwm = int(1000 - (right_speed_pct * 1000))

        # Handle Reverse 
        if v_x < 0:
            left_pwm = -left_pwm
            right_pwm = -right_pwm

        self.l_pwm_pub.publish(Int32(data=left_pwm))
        self.r_pwm_pub.publish(Int32(data=right_pwm))

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