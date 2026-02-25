import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import sys

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.bridge = CvBridge()
        
        # Create subscription for raw images using SensorDataQoS
        self.sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data 
        )
        
        # Create display window
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
        self.get_logger().info("Video subscriber started - press 'q' to exit")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Display the frame immediately
            cv2.imshow("Camera Feed", cv_image)
            
            # Check for exit key
            key = cv2.waitKey(1)
            if key in [ord('q'), ord('Q')]:
                self.get_logger().info("Shutting down by user request")
                # Trigger a clean exit
                raise KeyboardInterrupt
                
        except Exception as e:
            self.get_logger().error(f"CV bridge exception: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()