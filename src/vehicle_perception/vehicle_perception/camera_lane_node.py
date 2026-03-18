import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time

class CameraLaneNode(Node):
    def __init__(self):
        super().__init__('camera_lane_node')
        
        # Parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30) 
        
        cam_index = self.get_parameter('video_device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        self.bridge = CvBridge()
        
        # Open Camera
        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
        
        # Force MJPEG format before setting resolution
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open USB camera')
            return
            
        # Publishers
        self.raw_pub = self.create_publisher(Image, '/camera/lane/image_raw', 1)
        self.comp_pub = self.create_publisher(CompressedImage, '/camera/lane/image_compressed', 1)
        
        # Multithreading Safety Variables
        self.should_exit = False
        self.latest_frame = None
        self.has_new_frame = False
        self.frame_lock = threading.Lock()
        
        # Start Hardware Capture Thread
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.start()
        
        # Timer for ROS Publishing
        period_ms = 1.0 / fps
        self.timer = self.create_timer(period_ms, self.publish_frame)
        
        self.get_logger().info(f"Multithreaded Camera active at {width}x{height} @ {fps}fps")

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
            
        # Convert BGRA to BGR
        if len(frame.shape) == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
        stamp = self.get_clock().now().to_msg()
            
        # Publish Raw Message
        raw_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = "camera_lane_link"
        self.raw_pub.publish(raw_msg)

        # Publish Compressed Message
        comp_msg = CompressedImage()
        comp_msg.header.stamp = stamp
        comp_msg.header.frame_id = "camera_lane_link"
        comp_msg.format = "jpeg"
        _, encoded_image = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        comp_msg.data = np.array(encoded_image).tobytes()
        self.comp_pub.publish(comp_msg)

    def destroy_node(self):
        """Ensures the hardware thread shuts down cleanly."""
        self.should_exit = True
        if self.capture_thread.is_alive():
            self.capture_thread.join()
        if hasattr(self, 'cap') and self.cap.isOpened():
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