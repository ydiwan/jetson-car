import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from dataclasses import dataclass
from typing import List
from ament_index_python.packages import get_package_share_directory

@dataclass
class Waypoint:
    """Simple data structure to hold waypoint information."""
    x: float
    y: float
    name: str = ""

class WaypointLoaderNode(Node):
    def __init__(self):
        super().__init__('waypoint_loader')

        # 1. Declare and Get Parameters
        self.declare_parameter('csv_file', 'coordinates.csv')
        self.declare_parameter('publish_frequency', 10.0)  # Hz
        self.declare_parameter('frame_id', 'map')

        csv_filename = self.get_parameter('csv_file').value
        publish_frequency = self.get_parameter('publish_frequency').value
        self.frame_id = self.get_parameter('frame_id').value

        # Dynamically build the path to the CSV file 
        try:
            package_share_directory = get_package_share_directory('vehicle_controller_py')
            self.csv_file = os.path.join(package_share_directory, 'config', csv_filename)
        except Exception as e:
            self.get_logger().error(f"Could not find package share directory: {e}")
            self.csv_file = csv_filename # Fallback to just the filename

        # 2. Create Publishers
        self.waypoints_publisher = self.create_publisher(PoseArray, 'waypoints', 10)
        self.waypoint_names_publisher = self.create_publisher(String, 'waypoint_names', 10)

        # 3. Load Waypoints
        self.waypoints = self.load_waypoints(self.csv_file)

        # 4. Setup Timer if Waypoints Loaded Successfully
        if self.waypoints:
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {self.csv_file}")
            
            # Calculate timer period in seconds
            period = 1.0 / publish_frequency if publish_frequency > 0 else 0.1
            self.timer = self.create_timer(period, self.publish_waypoints)
        else:
            if self.csv_file:
                self.get_logger().error(f"No waypoints loaded from {self.csv_file}")
            else:
                self.get_logger().error("No CSV file specified. Use the 'csv_file' parameter.")

    def load_waypoints(self, filename: str) -> List[Waypoint]:
        """Reads the coordinates.csv file and parses the X, Y, and optional name."""
        waypoints = []
        if not filename:
            return waypoints

        try:
            with open(filename, 'r') as f:
                self.get_logger().info(f"Loading waypoints from: {filename}")
                
                for line in f:
                    line = line.strip()
                    
                    # Skip comments or empty lines
                    if not line or line.startswith('#'):
                        continue

                    parts = line.split(',')
                    
                    # Ensure we have at least X and Y
                    if len(parts) >= 2:
                        try:
                            x = float(parts[0].strip()) / 1000.0
                            y = float(parts[1].strip()) / 1000.0
                            
                            # Safely extract name if it exists, otherwise leave empty
                            name = parts[2].strip() if len(parts) > 2 else ""
                            
                            waypoints.append(Waypoint(x=x, y=y, name=name))
                        except ValueError:
                            # Skip malformed lines where conversion to float fails
                            continue 
                            
        except Exception as e:
            self.get_logger().error(f"Failed to open file: {filename}. Error: {e}")

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints")
        return waypoints

    def publish_waypoints(self):
        """Timer callback to publish the PoseArray and string of names."""
        if not self.waypoints:
            return

        # Prepare PoseArray message
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = self.frame_id

        names_list = []

        # Populate messages
        for wp in self.waypoints:
            pose = Pose()
            pose.position.x = wp.x
            pose.position.y = wp.y
            pose.position.z = 0.0

            # Set orientation to identity quaternion (no rotation)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            pose_array_msg.poses.append(pose)
            
            # Store name (or "unnamed" if empty)
            names_list.append(wp.name if wp.name else "unnamed")

        # Prepare String message (comma-separated names)
        names_msg = String()
        names_msg.data = ",".join(names_list)

        # Publish
        self.waypoints_publisher.publish(pose_array_msg)
        self.waypoint_names_publisher.publish(names_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()