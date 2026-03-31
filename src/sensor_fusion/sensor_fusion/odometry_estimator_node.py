import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time

class OdometryEstimatorNode(Node):
    def __init__(self):
        super().__init__('odometry_estimator_node')
        
        # State Variables 
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.v_x = 0.0
        self.omega_z = 0.0
        
        self.last_time = self.get_clock().now()

        #  Subscriptions 
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publishers 
        self.odom_pub = self.create_publisher(Odometry, '/vehicle/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Integration Timer (50Hz) 
        self.timer = self.create_timer(0.02, self.update_odometry)
        
        self.get_logger().info("Open-Loop Odometry Estimator Initialized.")

    def cmd_callback(self, msg: Twist):
        # Update our current assumed velocities based on the commands
        self.v_x = msg.linear.x
        self.omega_z = msg.angular.z

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Integrate position (Dead Reckoning)
        delta_x = (self.v_x * math.cos(self.theta)) * dt
        delta_y = (self.v_x * math.sin(self.theta)) * dt
        
        # A car cannot turn if it is not moving forward or backward.
        if abs(self.v_x) < 0.001:
            delta_theta = 0.0 
        else:
            delta_theta = self.omega_z * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish the Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert Theta (yaw) to Quaternion
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Set Velocity
        odom_msg.twist.twist.linear.x = self.v_x
        odom_msg.twist.twist.angular.z = self.omega_z
        
        # Add a baseline uncertainty to the velocities (Twist)
        odom_msg.twist.covariance[0] = 0.1  
        odom_msg.twist.covariance[35] = 0.1

        self.odom_pub.publish(odom_msg)

        # Broadcast the transform from odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()