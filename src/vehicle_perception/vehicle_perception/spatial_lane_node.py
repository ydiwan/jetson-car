import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct

class SpatialLaneNode(Node):
    def __init__(self):
        super().__init__('spatial_lane_node')
        self.bridge = CvBridge()
        
        # Camera Intrinsics & Extrinsics (1280x720 Gazebo Camera)
        self.cam_h = 0.0587
        self.cx = 640.0   
        self.cy = 0.0     # Horizon is exactly at the crop line
        self.fx = 1043.8
        self.fy = 1043.8
        
        # Subscribers and Publishers
        self.sub = self.create_subscription(Image, '/camera/lane/raw_video', self.image_callback, 1)
        
        self.pc_pub = self.create_publisher(PointCloud2, '/perception/lane_pointcloud', 1)
        self.debug_pub = self.create_publisher(Image, '/perception/spatial_debug', 1)
        
        self.get_logger().info("Spatial Lane Perception Node Initialized. Ready for Nav2.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        h, w = frame.shape[:2]
        
        # 1. Crop to ROI (Bottom half of the image)
        # Because pitch is 0, the physical horizon is exactly at row 360.
        roi = frame[int(h/2):h, :] 
        
        # 2. Color Thresholding (White and Yellow)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        lower_white = np.array([0, 0, 130])
        upper_white = np.array([180, 80, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        bin_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Morphological Open to delete tiny noise specs
        kernel = np.ones((3, 3), np.uint8)
        bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_OPEN, kernel)

        # 3. The 2.5m Lookahead Cap (Kills intersection stop lines)
        # By erasing the top 13 rows of the ROI, we prevent the math from 
        # looking further than ~2.5 meters down the road.
        bin_mask[0:13, :] = 0
        
        # Publish the black/white mask so you can debug the raw vision layer
        if self.debug_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(bin_mask, "mono8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

        # 4. Downsample to save Jetson CPU cycles for Nav2
        sampled_mask = np.zeros_like(bin_mask)
        sampled_mask[::10, ::10] = bin_mask[::10, ::10]
        
        v_coords, u_coords = np.nonzero(sampled_mask)
        
        if len(v_coords) == 0:
            return

        # 5. Vectorized Inverse Perspective Mapping (IPM)
        X = self.cam_h * self.fy / v_coords 
        Y = X * (self.cx - u_coords) / self.fx
        Z = np.full_like(X, -self.cam_h)

        # 6. Lateral Safety Crop (Keeps the corridor tightly bound to the lane width)
        valid_mask = np.abs(Y) < 1.0
        X_clean = X[valid_mask]
        Y_clean = Y[valid_mask]
        Z_clean = Z[valid_mask]

        if len(X_clean) == 0:
            return

        # 7. Publish PointCloud2
        points = np.vstack((X_clean, Y_clean, Z_clean)).T
        self.publish_pointcloud(points, msg.header.stamp)

    def publish_pointcloud(self, points, stamp):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_link'
        
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        
        buffer = [struct.pack('fff', x, y, z) for x, y, z in points]
        msg.data = b''.join(buffer)
        
        self.pc_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpatialLaneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()