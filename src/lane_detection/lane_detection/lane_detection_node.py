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
        
        # Initialize
        self.ld = LaneDetector(self) 

        # Setup OpenCV to ROS Image Bridge
        self.bridge = CvBridge()

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/lane/image_raw', 
            self.image_callback, 
            10
        )

        # Setup Publishers
        self.result_img_pub = self.create_publisher(Image, 'camera/lane_detection_result', 1)
        self.bev_img_pub = self.create_publisher(Image, 'camera/bev', 1)
        self.mask_img_pub = self.create_publisher(Image, 'camera/bin_mask', 1)
        self.filter_img_pub = self.create_publisher(Image, 'camera/filter_bin_mask', 1)
        
        # Detection publishers
        self.delta_pub = self.create_publisher(Int32, '/perception/lane_delta', 1)
        self.position_confidence_pub = self.create_publisher(Float64, 'lane_detect/position_confidence', 1)
        self.symmetry_confidence_pub = self.create_publisher(Float64, 'lane_detect/symmetry_confidence', 1)

        self.get_logger().info('Lane Detection Node has started.')

    def pub_img(self, publisher, img, encoding):
        """Helper to convert cv2 image to ROS Image msg and publish."""
        # Prevent chrashes if no frame is sent
        if img is None:
            return
        
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding=encoding)
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_lane_link'
            publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image back to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        if frame is not None:
            # Convert BGRA to BGR if necessary
            if len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            # Process frame
            results = self.ld.step(frame)
            
            # Create and publish data messages based on results
            msg_delta = Int32(data=int(results.delta))
            msg_pos_conf = Float64(data=float(results.position_conf))
            msg_sym_conf = Float64(data=float(results.symmetry_conf))
            
            self.delta_pub.publish(msg_delta)
            self.position_confidence_pub.publish(msg_pos_conf)
            self.symmetry_confidence_pub.publish(msg_sym_conf)

            # Publish results
            self.pub_img(self.result_img_pub, results.ld_img, 'bgr8')
            self.pub_img(self.bev_img_pub, results.bev, 'bgr8')
            self.pub_img(self.mask_img_pub, results.bin_mask, 'mono8')
            self.pub_img(self.filter_img_pub, results.filter_mask, 'mono8')

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