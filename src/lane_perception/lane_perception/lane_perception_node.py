import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float64
from cv_bridge import CvBridge
import cv2
from .lane_detector import LaneDetector

class LanePerceptionNode(Node):
    def __init__(self):
        super().__init__('lane_perception_node')
        
        # Initialize Core Components
        self.ld = LaneDetector(self) 
        self.bridge = CvBridge()

        # Declare Parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 60)
        self.declare_parameter('use_sim', False)

        cam_index = self.get_parameter('camera_index').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        use_sim = self.get_parameter('use_sim').value

        # Publishers
        self.raw_video_pub = self.create_publisher(Image, 'camera/lane/raw_video', 1)
        self.result_img_pub = self.create_publisher(Image, 'camera/lane_perception_result', 1)
        self.bev_img_pub = self.create_publisher(Image, 'camera/bev', 1)
        self.mask_img_pub = self.create_publisher(Image, 'camera/bin_mask', 1)
        self.filter_img_pub = self.create_publisher(Image, 'camera/filter_bin_mask', 1)
        
        self.delta_pub = self.create_publisher(Int32, 'lane_detect/delta', 1)
        self.position_confidence_pub = self.create_publisher(Float64, 'lane_detect/position_confidence', 1)
        self.symmetry_confidence_pub = self.create_publisher(Float64, 'lane_detect/symmetry_confidence', 1)

        # Simulation Param Handling
        if use_sim:
            self.get_logger().info('Simulation mode: Subscribing to Gazebo camera topic')
            # In sim, use gazebo camera
            self.image_sub = self.create_subscription(
                Image, 'camera/lane/raw_video', self.image_callback, 1)
        else:
            self.get_logger().info('Hardware mode: Opening USB camera')
            self.cap = cv2.VideoCapture(cam_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

            if not self.cap.isOpened():
                self.get_logger().error('Failed to open USB camera')
                return

            timer_period = 1.0 / fps
            self.timer = self.create_timer(timer_period, self.process_hardware_frame)

        self.get_logger().info('lane_perception_node has initialized')

    def image_callback(self, msg):
        """Callback for simulation camera feed."""
        try:
            # Convert ROS Image to OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_lane_logic(cv_image)
        except Exception as e:
            self.get_logger().error(f'Could not convert sim image: {e}')

    def process_hardware_frame(self):
        """Timer callback for physical camera feed."""
        ret, frame = self.cap.read()
        if ret and frame is not None:
            # Convert BGRA to BGR if necessary
            if len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            # Publish the raw frame so it matches the sim behavior
            self.pub_img(self.raw_video_pub, frame, 'bgr8')
            
            self.process_lane_logic(frame)

    def process_lane_logic(self, frame):
        """Unified perception and publishing logic used by both Sim and Hardware."""
        results = self.ld.step(frame)
        
        # Publish Data
        self.delta_pub.publish(Int32(data=int(results.delta)))
        self.position_confidence_pub.publish(Float64(data=float(results.position_conf)))
        self.symmetry_confidence_pub.publish(Float64(data=float(results.symmetry_conf)))

        # Publish Debug Image Streams
        self.pub_img(self.result_img_pub, results.ld_img, 'bgr8')
        self.pub_img(self.bev_img_pub, results.bev, 'bgr8')
        self.pub_img(self.mask_img_pub, results.bin_mask, 'mono8')
        self.pub_img(self.filter_img_pub, results.filter_mask, 'mono8')

    def pub_img(self, publisher, img, encoding):
        """Helper to convert cv2 image to ROS Image msg and publish."""
        if img is None:
            return
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding=encoding)
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LanePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()