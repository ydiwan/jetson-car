import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Subscriptions & Publishers
        self.image_sub = self.create_subscription(Image, '/camera/lane/image_raw', self.image_callback, 10)
        self.delta_pub = self.create_publisher(Int32, '/perception/lane_delta', 10)
        
        # A debug topic to tune
        self.debug_pub = self.create_publisher(Image, '/perception/lane_debug', 10)
        
        self.get_logger().info("Lane Detector Node Initialized. Waiting for frames...")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        height, width = frame.shape[:2]

        # Define the Bird's-Eye View perspective warp points
        # This draws a trapezoid on the original image targeting the lane lines.
        src = np.float32([
            [int(width * 0.1), height],             # Bottom-Left
            [int(width * 0.35), int(height * 0.6)], # Top-Left
            [int(width * 0.65), int(height * 0.6)], # Top-Right
            [int(width * 0.9), height]              # Bottom-Right
        ])
        
        # Destination points form a perfect top-down rectangle
        dst = np.float32([
            [int(width * 0.25), height], 
            [int(width * 0.25), 0], 
            [int(width * 0.75), 0], 
            [int(width * 0.75), height]
        ])

        # Calculate the transformation matrix and apply it
        matrix = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(frame, matrix, (width, height))

        # Find the white/yellow lines
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        
        # Filter
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

        # Calculate the Histogram to find the lanes
        histogram = np.sum(thresh[int(height/2):, :], axis=0)
        
        # Find the X-coordinate of the left and right lane peaks
        midpoint = int(histogram.shape[0] / 2)
        left_x_base = np.argmax(histogram[:midpoint])
        right_x_base = np.argmax(histogram[midpoint:]) + midpoint

        # Calculate the Delta
        lane_center = int((left_x_base + right_x_base) / 2)
        
        # The car's camera is in the middle of the frame
        car_center = int(width / 2)
        
        # Delta = where the car is vs where it should be
        delta = lane_center - car_center

        # Publish the delta for the control node
        delta_msg = Int32()
        delta_msg.data = delta
        self.delta_pub.publish(delta_msg)

        # Generate Debug Visuals (Temp)
        if self.debug_pub.get_subscription_count() > 0:
            cv2.line(warped, (lane_center, 0), (lane_center, height), (0, 255, 0), 3)
            cv2.line(warped, (car_center, 0), (car_center, height), (0, 0, 255), 3)
            
            debug_msg = self.bridge.cv2_to_imgmsg(warped, encoding="bgr8")
            self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()