import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
import math
import time

class ViconSubscriber(Node):
    def __init__(self):
        super().__init__('vicon_display_node')

        # Create subscription with the low-latency sensor data QoS profile
        self.subscription = self.create_subscription(
            PoseStamped,
            'vicon_pose',
            self.topic_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('Vicon position subscriber started')
        
        # Initialize timing variables for latency measurement (if needed later)
        self.last_message_time = time.perf_counter()

    def euler_from_quaternion(self, q):
        """Helper to convert a quaternion message into Euler angles (roll, pitch, yaw) in radians."""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def topic_callback(self, msg: PoseStamped):
        # Extract position data
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Convert quaternion to euler angles
        roll_rad, pitch_rad, yaw_rad = self.euler_from_quaternion(msg.pose.orientation)

        # Convert from radians to degrees
        roll = math.degrees(roll_rad)
        pitch = math.degrees(pitch_rad)
        yaw = math.degrees(yaw_rad)

        # Print position data with minimal formatting (scaled to mm as in original C++)
        print(
            f"Position: {x * 1000:8.2f} {y * 1000:8.2f} {z * 1000:8.2f} | "
            f"Orientation (deg): Roll={roll:8.3f} Pitch={pitch:8.3f} Yaw={yaw:8.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ViconSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()