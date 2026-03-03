import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float64
from cv_bridge import CvBridge
import cv2
from .lane_detector import LaneDetector

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Initialize your custom lane detector logic
        self.ld = LaneDetector(self) 

        # 1. Declare and Get Parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 60)

        cam_index = self.get_parameter('camera_index').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        # 2. Setup OpenCV to ROS Image Bridge
        self.bridge = CvBridge()

        # 3. Create and Open GStreamer Pipeline
        pipeline = self.create_pipeline(cam_index, width, height, fps)
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera with pipeline: {pipeline}')
            return

        # 4. Setup Publishers
        self.raw_video_pub = self.create_publisher(Image, 'camera/raw_video', 1)
        self.result_img_pub = self.create_publisher(Image, 'camera/lane_detection_result', 1)
        self.bev_img_pub = self.create_publisher(Image, 'camera/bev', 1)
        self.mask_img_pub = self.create_publisher(Image, 'camera/bin_mask', 1)
        self.filter_img_pub = self.create_publisher(Image, 'camera/filter_bin_mask', 1)
        
        self.delta_pub = self.create_publisher(Int32, 'lane_detect/delta', 1)
        self.position_confidence_pub = self.create_publisher(Float64, 'lane_detect/position_confidence', 1)
        self.symmetry_confidence_pub = self.create_publisher(Float64, 'lane_detect/symmetry_confidence', 1)

        self.get_logger().info('lane_detection_node has started')

        # 5. Create Timer to trigger frame processing
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.process_frame)

    def pub_img(self, publisher, img, encoding):
        """Helper to convert cv2 image to ROS Image msg and publish."""
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding=encoding)
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'video_feed'
            publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')

    def process_frame(self):
        ret, frame = self.cap.read()

        if ret and frame is not None:
            # Convert BGRA to BGR if necessary
            if len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            # --- The custom processing step ---
            results = self.ld.step(frame)
            
            # Create and publish data messages based on results
            msg_delta = Int32(data=int(results.delta))
            msg_pos_conf = Float64(data=float(results.position_conf))
            msg_sym_conf = Float64(data=float(results.symmetry_conf))
            
            self.delta_pub.publish(msg_delta)
            self.position_confidence_pub.publish(msg_pos_conf)
            self.symmetry_confidence_pub.publish(msg_sym_conf)

            # Publish image streams
            self.get_logger().debug('pub images')
            self.pub_img(self.raw_video_pub, frame, 'bgr8')
            
            # Uncomment when your LaneDetector is ready:
            self.pub_img(self.result_img_pub, results.ld_img, 'bgr8')
            self.pub_img(self.bev_img_pub, results.bev, 'bgr8')
            self.pub_img(self.mask_img_pub, results.bin_mask, 'mono8')
            self.pub_img(self.filter_img_pub, results.filter_mask, 'mono8')

    def create_pipeline(self, cam_index, width, height, fps):
        """Constructs the GStreamer pipeline string for Jetson."""
        pipeline = (
            f"nvarguscamerasrc sensor-id={cam_index} "
            f"! video/x-raw(memory:NVMM),width=(int){width},height=(int){height},framerate=(fraction){fps}/1,format=NV12 "
            f"! nvvidconv "
            f"! video/x-raw,width=(int){width},height=(int){height},format=(string)BGRx "
            f"! videoconvert "
            f"! video/x-raw,format=BGR "
            f"! appsink sync=false"
)
        self.get_logger().info(f'Pipeline: {pipeline}')
        return pipeline

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()