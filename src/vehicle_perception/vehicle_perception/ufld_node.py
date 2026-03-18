import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
import scipy.special
import sys
import os

# Point this to exactly where the UFLD repo lives on your Jetson
UFLD_PATH = '/home/youssef/rework/Ultra-Fast-Lane-Detection-v2'
sys.path.append(UFLD_PATH)

from utils.config import Config
from utils.factory import get_model

class UFLDNode(Node):
    def __init__(self):
        super().__init__('ufld_perception_node')
        self.bridge = CvBridge()
        
        self.get_logger().info("Initializing UFLD ResNet18 on Orin GPU...")
        
        # Setup PyTorch Device
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        
        # Load UFLD Config for CurveLanes
        config_path = os.path.join(UFLD_PATH, 'configs/curvelanes_res18.py')
        self.cfg = Config.fromfile(config_path)
        
        # Construct the Model using UFLD's Factory
        self.net = get_model(self.cfg)
        
        # Load the Weights
        weights_path = os.path.join(UFLD_PATH, 'curvelanes_resnet18.pth')
        state_dict = torch.load(weights_path, map_location=self.device)
        self.net.load_state_dict(state_dict['model'])
        self.net.to(self.device)
        self.net.eval() # Set to evaluation mode!

        # Image Preprocessing Pipeline
        self.img_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])

        # ROS Communications
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.delta_pub = self.create_publisher(Int32, 'lane_detect/delta', 1)
        self.conf_pub = self.create_publisher(Float64, 'lane_detect/position_confidence', 1)
        self.debug_pub = self.create_publisher(Image, '/camera/ufld_debug', 1)

        self.get_logger().info("AI Perception Pipeline is LIVE.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        original_h, original_w = frame.shape[:2]

        # Preprocess
        img_resized = cv2.resize(frame, (self.cfg.train_width, self.cfg.train_height))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        tensor_img = self.img_transform(img_rgb).unsqueeze(0).to(self.device)

        # Inference
        with torch.no_grad(): 
            out = self.net(tensor_img)

        # Parse output
        # UFLD v2 parsing logic to extract X coordinates
        col_sample = np.linspace(0, self.cfg.train_width - 1, self.cfg.num_points)
        col_sample_w = col_sample[1] - col_sample[0]
        out_j = out[0].data.cpu().numpy()
        out_j = out_j[:, ::-1, :]
        prob = scipy.special.softmax(out_j[:-1, :, :], axis=0)
        idx = np.arange(self.cfg.num_points) + 1
        idx = idx.reshape(-1, 1, 1)
        loc = np.sum(prob * idx, axis=0)
        out_j = np.argmax(out_j, axis=0)
        loc[out_j == self.cfg.num_points] = 0
        out_j = loc
        
        # Scale the coordinates back up to 1280x720 camera resolution
        width_ratio = original_w / self.cfg.train_width
        
        # Find the bottom-most points of the detected lanes
        # CurveLanes detects up to 14 lines, only get the two closest to the center
        valid_lanes_x = []
        for i in range(out_j.shape[1]):
            bottom_x = out_j[-1, i] # Grab the X coordinate at the bottom row
            if bottom_x > 0:
                valid_lanes_x.append(bottom_x * width_ratio)

        valid_lanes_x.sort()
        
        # Default fallbacks
        left_x = -1
        right_x = -1
        confidence = 0.1
        image_center = original_w / 2

        # Identify the immediate left and right lanes relative to the car's center
        for x in valid_lanes_x:
            if x < image_center:
                left_x = x
            elif x > image_center and right_x == -1:
                right_x = x

        # Calculate steering delta
        if left_x != -1 and right_x != -1:
            lane_center = (left_x + right_x) / 2
            delta = int(lane_center - image_center)
            confidence = 0.95
        elif left_x != -1: # Missing right line fallback
            lane_center = left_x + 450 # Actual pixel length of the road
            delta = int(lane_center - image_center)
            confidence = 0.60
        elif right_x != -1: # Missing left line fallback
            lane_center = right_x - 450 
            delta = int(lane_center - image_center)
            confidence = 0.60
        else:
            delta = 0

        # Publishers
        self.delta_pub.publish(Int32(data=delta))
        self.conf_pub.publish(Float64(data=confidence))

        # Debugging
        if self.debug_pub.get_subscription_count() > 0:
            if left_x != -1:
                cv2.circle(frame, (int(left_x), original_h - 50), 15, (255, 0, 0), -1)
            if right_x != -1:
                cv2.circle(frame, (int(right_x), original_h - 50), 15, (0, 0, 255), -1)
            
            cv2.circle(frame, (int(image_center + delta), original_h - 50), 15, (0, 255, 0), -1)
            
            debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UFLDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()