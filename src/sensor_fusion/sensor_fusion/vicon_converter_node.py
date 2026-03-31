import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class ViconConverter(Node):
    def __init__(self):
        super().__init__('vicon_converter_node')
        
        # Subscribe to the odom
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/jetson_car1/vicon_pose', 10)

    def callback(self, msg):
        new_msg = PoseWithCovarianceStamped()
        new_msg.header = msg.header
        
        new_msg.pose.pose = msg.pose.pose
        
        # Add the vicon covariance
        new_msg.pose.covariance[0] = 0.01  # X
        new_msg.pose.covariance[7] = 0.01  # Y
        new_msg.pose.covariance[14] = 0.01 # Z
        
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ViconConverter())
    rclpy.shutdown()

if __name__ == '__main__':
    main()