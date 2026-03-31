import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimGroundTruthNode(Node):
    def __init__(self):
        super().__init__('sim_ground_truth_node')
        
        self.sub = self.create_subscription(Odometry, '/model/jetson_car/odometry', self.odom_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.received_odom = False
        
        # 50Hz Kickstart to keep Nav2 happy while Gazebo boots
        self.timer = self.create_timer(0.02, self.kickstart_cb)
        self.get_logger().info("Sim Ground Truth Vicon-Emulator Active. Waiting for Gazebo...")

    def kickstart_cb(self):
        if not self.received_odom:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
        else:
            self.timer.cancel()

    def odom_cb(self, msg):
        if not self.received_odom:
            self.get_logger().info("SUCCESS: Received Ground Truth from Gazebo! Synchronizing TF Tree.")
            self.received_odom = True
            
        # Forward Odometry to Nav2
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.odom_pub.publish(msg)

        # Broadcast Flawless TF Transform
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SimGroundTruthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()