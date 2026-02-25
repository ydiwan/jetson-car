import cv2
import numpy as np
import rclpy
from .ld_config import LdConfig # Assumes ld_config is available in the same folder

class LdPostprocessor:
    def __init__(self, config: LdConfig, img_width: int, logger=None):
        self.config = config
        self.logger = logger or rclpy.logging.get_logger('Ld_postprocessor')

        # ============== Bird View Transformation Variables ============================
        self.bev_left_lane = []
        self.bev_right_lane = []
        
        # Source pixel coordinates of road image to warp from
        src_points = np.float32([
            self.config.top_left_src,
            self.config.top_right_src,
            self.config.btm_left_src,
            self.config.btm_right_src
        ])

        # Destination points to warp the source points to
        top_left_dst = [0.30 * img_width, self.config.top_left_src[1]]
        bottom_left_dst = [0.30 * img_width, self.config.btm_left_src[1]]
        top_right_dst = [0.70 * img_width, self.config.top_right_src[1]]
        bottom_right_dst = [0.70 * img_width, self.config.btm_left_src[1]]

        dst_points = np.float32([
            top_left_dst,
            top_right_dst,
            bottom_left_dst,
            bottom_right_dst
        ])

        # Create transformation matrix
        self.trans = cv2.getPerspectiveTransform(src_points, dst_points)

        # Calculate center of the image and apply offset
        center = img_width / 2.0
        self.center_of_image = int(center + (center * self.config.center_offset))

        # ============== Image Buffers ================================================
        self.ld_result = None
        self.bev = None

    def run(self, rgb_img: np.ndarray, left_lane: list, right_lane: list, center_lane: list) -> int:
        """
        Main processing function for post-processing lane detection results.
        Returns the delta (horizontal pixel offset) for steering.
        """
        # Shallow copy the rgb image for drawing
        self.ld_result = rgb_img.copy()

        # Add Visualization to ld_result
        self.logger.debug(" Left Edge | Center | Right Edge ")
        self.logger.debug("----+----------+-----------+--------")
        
        for i in range(len(center_lane)):
            # Mark circles on the image (using the tuples from ld_scanner)
            cv2.circle(self.ld_result, left_lane[i], 1, (255, 0, 0), -1)   # Blue: Left
            cv2.circle(self.ld_result, right_lane[i], 1, (0, 255, 0), -1)  # Green: Right
            cv2.circle(self.ld_result, center_lane[i], 1, (0, 0, 255), -1) # Red: Center
            
            self.logger.debug(f"{left_lane[i]} | {center_lane[i]} | {right_lane[i]}")

        # Get the target pixel position we want the car to align itself with
        if not center_lane:
            self.logger.warn("No center lane found, returning delta 0")
            return 0
            
        target = center_lane[-1]
        self.logger.debug(f"target: ({target[0]}, {target[1]})")

        # Calculate delta
        delta = target[0] - self.center_of_image
        self.logger.debug(f"delta: {delta}")

        # Do BEV transformation
        self.bev_trans(self.ld_result, left_lane, right_lane)

        return delta

    def bev_trans(self, rgb_img: np.ndarray, left_lane: list, right_lane: list):
        """
        Warp the road image to Bird's Eye View and transform the lane coordinates.
        """
        height, width = rgb_img.shape[:2]
        
        # Warp the image
        self.bev = cv2.warpPerspective(rgb_img, self.trans, (width, height))

        # If we have lane points, warp them too
        if left_lane and right_lane:
            # OpenCV perspectiveTransform requires a float32 array of shape (N, 1, 2)
            left_float = np.array(left_lane, dtype=np.float32).reshape(-1, 1, 2)
            right_float = np.array(right_lane, dtype=np.float32).reshape(-1, 1, 2)

            self.bev_left_lane = cv2.perspectiveTransform(left_float, self.trans)
            self.bev_right_lane = cv2.perspectiveTransform(right_float, self.trans)
        else:
            self.bev_left_lane = []
            self.bev_right_lane = []

    # Getters
    def get_bev_left_lane(self): return self.bev_left_lane
    def get_bev_right_lane(self): return self.bev_right_lane
    def get_bev_img(self): return self.bev
    def get_ld_result(self): return self.ld_result