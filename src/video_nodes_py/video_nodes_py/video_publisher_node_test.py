import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 60)
        
        cam_index = self.get_parameter('camera_index').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        self.bridge = CvBridge()
        
        # open gstreamer
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera with pipeline: {pipeline}")
            return
            
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 1)
        
        # safe frame buffer variables
        self.should_exit = False
        self.latest_frame = None
        self.has_new_frame = False
        self.frame_lock = threading.Lock()
        
        # start hardware capture thread
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.start()
        
        # timer for publishing
        period_ms = 1.0 / fps
        self.timer = self.create_timer(period_ms, self.publish_frame)
        
        self.get_logger().info(f"Video publisher started at {fps} fps")
        self.get_logger().info("Publishing on: camera/image_raw")

    def capture_loop(self):
        """Continuously pulls frames from the hardware buffer to prevent lag."""
        while not self.should_exit and rclpy.ok():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                    self.has_new_frame = True
            else:
                time.sleep(0.01)

    def publish_frame(self):
        """Grabs the newest frame from the thread and publishes it to ROS."""
        with self.frame_lock:
            if not self.has_new_frame:
                return  # No new frame available
            frame = self.latest_frame.copy()
            self.has_new_frame = False
            
        # Convert BGRx to BGR if necessary
        if len(frame.shape) == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
        # Create and publish raw image message
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        
        self.image_pub.publish(msg)

    def destroy_node(self):
        """Ensures the hardware thread shuts down cleanly."""
        self.should_exit = True
        if self.capture_thread.is_alive():
            self.capture_thread.join()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()