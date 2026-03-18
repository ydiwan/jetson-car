import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class SteeringBridge(Node):
    def __init__(self):
        super().__init__('steering_bridge_node')
        
        # Subscription
        self.delta_sub = self.create_subscription(Int32, 'lane_detect/delta', self.delta_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Forward speed
        self.declare_parameter('speed', 0.3) 
        # Proportional Gain 
        self.declare_parameter('kp', 0.005)       

        self.speed = self.get_parameter('speed').value
        self.kp = self.get_parameter('kp').value
        
        self.get_logger().info(f"Steering Bridge engaged. Speed: {self.speed}, Kp: {self.kp}")

    def delta_callback(self, msg):
        pixel_error = msg.data
        cmd = Twist()

        # Constant forward drive
        cmd.linear.x = float(self.speed)
        
        # Calculate steering angle based on pixel error
        cmd.angular.z = float(-self.kp * pixel_error)

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop before shutting down
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()