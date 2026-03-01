import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Int32, Float64
import time

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('wp_follower')

        # 1. Subscriptions to monitor the car's state
        self.vicon_sub = self.create_subscription(
            PoseStamped, 
            'vicon_pose', 
            self.vicon_callback, 
            10
        )
        
        self.waypoint_sub = self.create_subscription(
            PoseArray, 
            'waypoints', 
            self.waypoint_callback, 
            10
        )

        # 2. Track internal state
        self.current_pose = None
        self.target_waypoints = []
        self.mission_started = False

        self.get_logger().info("Waypoint Follower (Mission Control) started.")

    def vicon_callback(self, msg):
        self.current_pose = msg.pose
        if not self.mission_started and len(self.target_waypoints) > 0:
            self.get_logger().info("Waypoints received. Mission is active.")
            self.mission_started = True

    def waypoint_callback(self, msg):
        self.target_waypoints = msg.poses
        self.get_logger().info(f"Synchronized {len(msg.poses)} waypoints from loader.")

    def run_mission(self):
        """
        High-level logic to monitor mission status.
        The actual steering math is handled by the vehicle_controller_node.
        """
        if self.mission_started:
            # You can add logic here to print progress or stop the car
            # if a specific global condition is met.
            pass

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down mission control.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()