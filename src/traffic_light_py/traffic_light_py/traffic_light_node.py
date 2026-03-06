import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')
        self.bridge = CvBridge()
        
        # Subscribe to the camera feed
        self.image_sub = self.create_subscription(Image, '/driver_perspective/image_raw', self.image_callback, 10)
        
        # Publishers for the car's brain and the web server
        self.state_pub = self.create_publisher(String, '/traffic_light_state', 10)
        self.debug_pub = self.create_publisher(Image, '/traffic_light_debug', 10)
        
        self.get_logger().info("Traffic Light Node Started! Waiting for images...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message back to an OpenCV frame
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # CROP: Look only at the top half for traffic lights
        height, width = frame.shape[:2]
        roi = frame[0:height//2, 0:width]

        # CONVERT & THRESHOLD: BGR to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define HSV color ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([90, 255, 255])

        # Create Masks
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Find Contours
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        light_state = "NONE"

        # Check Red
        for cnt in contours_red:
            if cv2.contourArea(cnt) > 100:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(roi, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(roi, "RED LIGHT", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                light_state = "RED"

        # Check Green
        for cnt in contours_green:
            if cv2.contourArea(cnt) > 100:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(roi, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(roi, "GREEN LIGHT", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                light_state = "GREEN"

        # Reattach the ROI and Publish
        frame[0:height//2, 0:width] = roi
        
        # Publish the state string
        state_msg = String()
        state_msg.data = light_state
        self.state_pub.publish(state_msg)
        
        # Publish the annotated image
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()