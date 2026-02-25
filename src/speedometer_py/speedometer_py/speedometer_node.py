import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from collections import deque
import numpy as np
import math

class SpeedometerNode(Node):
    def __init__(self):
        super().__init__('speedometer')

        # 1. Setup Subscription and Publisher
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'vicon_pose',
            self.cal_velocities,
            2
        )
        self.velocity_pub = self.create_publisher(Twist, 'velocity', 1)

        # 2. Declare and Get Parameters
        self.declare_parameter('median_window_size', 5)
        self.declare_parameter('median_threshold', 2.0)  # rad/s

        self.median_window_size = self.get_parameter('median_window_size').value
        self.median_threshold = self.get_parameter('median_threshold').value

        # 3. Initialize State Variables
        self.last_pose_msg = None
        # Using deque with maxlen automatically handles popping old values!
        self.angular_velo_history = deque(maxlen=self.median_window_size)
        self.VICON_DT = 0.01  # 100Hz from Vicon

        self.get_logger().info(
            f"Speedometer_node has started with median filter "
            f"(window={self.median_window_size}, threshold={self.median_threshold:.2f} rad/s)"
        )

    def apply_median_filter(self, new_value: float) -> float:
        """Applies a median filter to reject outlier angular velocity spikes."""
        self.angular_velo_history.append(new_value)

        # Need at least 3 values for meaningful median
        if len(self.angular_velo_history) < 3:
            return new_value

        median = float(np.median(self.angular_velo_history))

        # Check if new value is an outlier (beyond threshold from median)
        if abs(new_value - median) > self.median_threshold:
            self.get_logger().debug(
                f"Angular velocity {new_value:.3f} rad/s rejected "
                f"(median={median:.3f}, diff={abs(new_value - median):.3f} > threshold={self.median_threshold:.3f})"
            )
            return median

        return new_value

    def get_yaw_from_quaternion(self, q) -> float:
        """Helper to convert a quaternion message into a yaw angle (radians)."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def cal_velocities(self, pose_msg):
        """Callback to calculate linear and angular velocities from pose data."""
        if self.last_pose_msg is None:
            self.last_pose_msg = pose_msg
            return

        # Calculate x and y linear velocities
        dx = pose_msg.pose.position.x - self.last_pose_msg.pose.position.x
        dy = pose_msg.pose.position.y - self.last_pose_msg.pose.position.y
        
        x_velo = dx / self.VICON_DT
        y_velo = dy / self.VICON_DT

        # Calculate angular velocity (yaw)
        current_yaw = self.get_yaw_from_quaternion(pose_msg.pose.orientation)
        last_yaw = self.get_yaw_from_quaternion(self.last_pose_msg.pose.orientation)

        delta_yaw = current_yaw - last_yaw

        # Assume the car took the shortest path to reach its current angle
        if delta_yaw > math.pi:
            delta_yaw -= 2 * math.pi
        elif delta_yaw < -math.pi:
            delta_yaw += 2 * math.pi

        angular_velo = delta_yaw / self.VICON_DT

        # Apply median filter
        filtered_angular_velo = self.apply_median_filter(angular_velo)

        # The vicon angle samples have jitter, round to tenths place (hundredths natively)
        filtered_angular_velo = round(filtered_angular_velo * 100) / 100.0

        # Publish the Velocity Message
        velocity_msg = Twist()
        velocity_msg.linear.x = float(x_velo)
        velocity_msg.linear.y = float(y_velo)
        velocity_msg.angular.z = float(filtered_angular_velo)

        self.velocity_pub.publish(velocity_msg)

        # Debug Logging
        raw_deg = round((angular_velo * (180.0 / math.pi)) * 100) / 100.0
        filt_deg = round((filtered_angular_velo * (180.0 / math.pi)) * 100) / 100.0
        
        self.get_logger().debug(
            f"X velocity: {x_velo:.3f} m/s, Y velocity: {y_velo:.3f} m/s, "
            f"Angular velocity: {filt_deg:.3f} degree/sec (raw: {raw_deg:.3f})"
        )

        self.last_pose_msg = pose_msg

def main(args=None):
    rclpy.init(args=args)
    node = SpeedometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()