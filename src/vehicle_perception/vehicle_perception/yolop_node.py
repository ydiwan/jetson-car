import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torchvision.transforms as transforms
import numpy as np

class YolopPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolop_perception_node')
        
        self.bridge = CvBridge()
        self.get_logger().info('Initializing YOLOP via PyTorch Hub (This may take a moment to download weights)...')

        # This automatically pulls the repo and pretrained weights into ~/.cache/torch/hub/
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.hub.load('hustvl/yolop', 'yolop', pretrained=True, trust_repo=True)
        self.model.to(self.device)
        self.model.eval()

        # Image transformations
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((640, 640)),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, 
            'camera/lane/raw_video', 
            self.image_callback, 
            1
        )
        
        # Publishers
        self.da_pub = self.create_publisher(Image, 'camera/yolop/drivable_area', 1)
        self.overlay_pub = self.create_publisher(Image, 'camera/yolop/overlay', 1)

        self.get_logger().info(f'YOLOP Node active and running on: {self.device}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            original_h, original_w = cv_image.shape[:2]

            # Convert to RGB for PyTorch
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Apply transforms and move to GPU
            img_tensor = self.transform(rgb_image).unsqueeze(0).to(self.device)

            # Inference
            with torch.no_grad():
                det_out, da_seg_out, ll_seg_out = self.model(img_tensor)

            # da_seg_out shape is (1, 2, 640, 640). Channel 0 is background, Channel 1 is drivable.
            da_predict = torch.argmax(da_seg_out, dim=1).squeeze().cpu().numpy()
            
            # Convert the 640x640 mask back to the original camera size
            da_mask_resized = cv2.resize(
                da_predict.astype(np.uint8), 
                (original_w, original_h), 
                interpolation=cv2.INTER_NEAREST
            )

            # Publish the raw binary mask
            binary_mask = da_mask_resized * 255
            self.pub_img(self.da_pub, binary_mask, 'mono8')

            # Create a Green Overlay on the original image for easy viewing
            overlay = cv_image.copy()
            # Where the mask is 1, turn the pixels green
            overlay[da_mask_resized == 1] = (0, 255, 0)
            
            # Blend the overlay with the original image (50% transparency)
            blended = cv2.addWeighted(cv_image, 0.5, overlay, 0.5, 0)
            self.pub_img(self.overlay_pub, blended, 'bgr8')

        except Exception as e:
            self.get_logger().error(f'YOLOP inference error: {e}')

    def pub_img(self, publisher, img, encoding):
        """Helper to convert cv2 image to ROS msg."""
        if img is None: return
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding=encoding)
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YolopPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()