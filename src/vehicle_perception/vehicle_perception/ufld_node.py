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

HOME = os.path.expanduser("~")
UFLD_FOLDER_NAME = 'Ultra-Fast-Lane-Detection-V2' 
UFLD_PATH = os.path.join(HOME, 'youssef/rework', UFLD_FOLDER_NAME)

if not os.path.exists(UFLD_PATH):
    print(f"CRITICAL ERROR: UFLD path not found at {UFLD_PATH}  ")
else:
    sys.path.append(UFLD_PATH)

try:
    from utils.config import Config
    from utils.common import get_model
except ImportError as e:
    print(f"IMPORT ERROR: Could not find UFLD modules. Check if {UFLD_PATH} contains 'utils' folder.")
    raise e

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
        weights_path = os.path.join(UFLD_PATH, 'curvelanes_res18.pth')
        state_dict = torch.load(weights_path, map_location=self.device)
        
        # Strip module prefix from multi-gpu training
        clean_state_dict = {}
        for k, v in state_dict['model'].items():
            name = k.replace('module.', '') if k.startswith('module.') else k
            clean_state_dict[name] = v
        
        self.net.load_state_dict(clean_state_dict)

        # Image Preprocessing Pipeline
        self.img_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])

        # ROS Communications
        self.image_sub = self.create_subscription(Image, '/camera/lane/raw_video', self.image_callback, 10)
        self.delta_pub = self.create_publisher(Int32, 'lane_detect/delta', 1)
        self.conf_pub = self.create_publisher(Float64, 'lane_detect/position_confidence', 1)
        self.debug_pub = self.create_publisher(Image, '/camera/lane_perception_result', 1)

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
        valid_lanes_x = []
        try:
            loc_row = out['loc_row']      
            exist_row = out['exist_row']  
            
            max_indices = loc_row.argmax(1)[0].cpu().numpy() 
            valid = exist_row.argmax(1)[0].cpu().numpy()     
            
            num_row_anchors, num_lanes = max_indices.shape
            num_grid_row = loc_row.shape[1]
            
            lookahead_start = int(num_row_anchors * 0.6) 
            
            for i in range(num_lanes):
                # Search from the lookahead point down towards the bumper
                for r in range(lookahead_start, num_row_anchors):
                    if valid[r, i] == 1:
                        grid_idx = max_indices[r, i]
                        x_coord = (grid_idx + 0.5) * (original_w / num_grid_row)
                        valid_lanes_x.append(x_coord)
                        break
        except Exception as e:
            self.get_logger().error(f"Parsing error: {e}")
            return

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

        lane_width_at_lookahead = 250
        
        # Calculate steering delta
        if left_x != -1 and right_x != -1:
            lane_center = (left_x + right_x) / 2
            delta = int(lane_center - image_center)
            confidence = 0.95
        elif left_x != -1: # Missing right line fallback
            lane_center = left_x + lane_width_at_lookahead # Actual pixel length of the road
            delta = int(lane_center - image_center)
            confidence = 0.60
        elif right_x != -1: # Missing left line fallback
            lane_center = right_x - lane_width_at_lookahead 
            delta = int(lane_center - image_center)
            confidence = 0.60
        else:
            delta = 0

        # Publishers
        self.delta_pub.publish(Int32(data=delta))
        self.conf_pub.publish(Float64(data=confidence))

        # Debug Drawing
        draw_y = int(original_h * 0.6)
        
        if self.debug_pub.get_subscription_count() > 0:
            if left_x != -1:
                cv2.circle(frame, (int(left_x), draw_y), 15, (255, 0, 0), -1)
            if right_x != -1:
                cv2.circle(frame, (int(right_x), draw_y), 15, (0, 0, 255), -1)
            
            cv2.circle(frame, (int(image_center + delta), draw_y), 15, (0, 255, 0), -1)
            
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