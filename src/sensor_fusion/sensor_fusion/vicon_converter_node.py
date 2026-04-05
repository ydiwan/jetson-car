import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data

class ViconTFBroadcaster(Node):
    def __init__(self):
        super().__init__('vicon_tf_broadcaster')
        
        # Initialize broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriptions
        self.subscription = self.create_subscription(
            PoseStamped,
            'vicon_pose', 
            self.vicon_callback,
            qos_profile_sensor_data)
            
        self.get_logger().info("Vicon TF Bridge active: Translating vicon_pose to odom -> base_link")

    def vicon_callback(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ViconTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()