import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraLaneNode(Node):
    def __init__(self):
        super().__init__('camera_lane_node')
        
        # Parameters
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30)

        self.device = self.get_parameter('video_device').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value

        # Initialize Publisher and Bridge
        self.publisher_ = self.create_publisher(Image, '/camera/lane/image_raw', 10)
        self.br = CvBridge()

        # Initialize OpenCV
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        
        # Apply Hardware Settings
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera at {self.device}!")
            raise RuntimeError("Camera connection failed.")
        else:
            self.get_logger().info(f"Camera opened at {self.width}x{self.height} @ {self.fps}FPS")

        # Capture Loop
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convert BGR) to ROS 2 Image message
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_lane_link"
            
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning("Dropped frame: Failed to read from camera.")

    def destroy_node(self):
        self.get_logger().info("Releasing camera hardware...")
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraLaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()