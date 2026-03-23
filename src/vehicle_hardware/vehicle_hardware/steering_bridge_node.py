import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class SteeringBridge(Node):
    def __init__(self):
        super().__init__('steering_bridge_node')
        
        # Subscription and Publisher
        self.delta_sub = self.create_subscription(Int32, 'lane_detect/delta', self.delta_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.declare_parameter('max_speed', 0.3) 
        self.declare_parameter('min_speed', 0.05)
        # self.declare_parameter('kp', 0.005)      # Steering strength
        # self.declare_parameter('kd', 0.002)      # Steering dampening
        self.declare_parameter('kp', 0.5)      # Steering strength
        self.declare_parameter('kd', 0.05)      # Steering dampening

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        
        # State variable for the Derivative calculation
        self.prev_error = 0.0
        
        self.get_logger().info(f"Dynamic Steering engaged. Max Speed: {self.max_speed}, Kp: {self.kp}, Kd: {self.kd}")

    def delta_callback(self, msg):
        pixel_error = float(msg.data)
        cmd = Twist()

        # PD Steering Control
        derivative = pixel_error - self.prev_error
        steering_angle = (-self.kp * pixel_error) + (-self.kd * derivative)
        self.prev_error = pixel_error

        # Clamp steering
        cmd.angular.z = max(-1.0, min(1.0, steering_angle))

        # Dynamic Speed Control
        # Calculate a speed penalty based on how sharp the turn is
        speed_penalty = abs(pixel_error) * 0.001 
        current_speed = self.max_speed - speed_penalty
        
        # Apply the speed, but never drop below min_speed to prevent stalling
        cmd.linear.x = max(self.min_speed, current_speed)

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()