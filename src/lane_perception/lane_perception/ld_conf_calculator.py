import rclpy
import math
import numpy as np
from .ld_config import LdConfig 

class LdConfCalculator:
    def __init__(self, config: LdConfig, center_of_image: int, logger=None):
        self.config = config
        self.center_of_image = center_of_image
        self.logger = logger or rclpy.logging.get_logger('Ld_conf_calculator')
        
        self.pos_conf = 0.0
        self.EXP_SCALE = -2.772588  # Scale for exponential confidence drop off

        self.sym_conf = 0.0

    def cal_position_conf(self, target_x: int) -> float:
        """
        Calculates how confident we are based on the target x position.
        Confidence drops exponentially as the target moves away from the center.
        """
        # Calculate percent difference between target x and center of image
        # Using abs() to handle negative drift
        avg_val = (self.center_of_image + target_x) / 2.0
        
        # Prevent division by zero
        if avg_val == 0:
            return 0.0
            
        per_diff = abs((self.center_of_image - target_x) / avg_val)

        if per_diff < self.config.pos_conf_threshold:
            confidence = 1.0  # High confidence if within threshold
        else:
            # confidence = e ^ (-2.772588 * percent difference).
            # confidence drops off exponentially as percent difference increases.
            confidence = math.exp(self.EXP_SCALE * per_diff)

        self.logger.debug(f"Position Confidence: {confidence}")
        self.pos_conf = confidence
        return confidence

    # def cal_symmetrical_conf(self, bev_left_lane, bev_right_lane) -> float:
    #     """
    #     Calculates symmetrical confidence based on left and right lane geometry in BEV.
    #     Returns a float between 0.0 and 1.0.
    #     """
    #     # bev_left_lane and bev_right_lane are expected to be the output from 
    #     # cv2.perspectiveTransform, which has shape (N, 1, 2).
    #     if bev_left_lane is None or bev_right_lane is None or len(bev_left_lane) == 0 or len(bev_right_lane) == 0:
    #         return 0.0

    #     # Squeeze the arrays from (N, 1, 2) to (N, 2) for easier coordinate access
    #     left_pts = np.squeeze(bev_left_lane)
    #     right_pts = np.squeeze(bev_right_lane)
        
    #     if left_pts.ndim == 1:
    #         left_pts = np.array([left_pts])
    #     if right_pts.ndim == 1:
    #         right_pts = np.array([right_pts])

    #     # If lengths don't match or there's only 1 point, we can't compute slopes
    #     if len(left_pts) != len(right_pts) or len(left_pts) < 2:
    #         return 0.0

    #     last_left_point = left_pts[0]
    #     last_right_point = right_pts[0]

    #     lowest_conf = float('inf')
    #     history_conf = []

    #     # Iterate through the points to calculate slope angles
    #     for i in range(1, len(left_pts)):
    #         left_point = left_pts[i]
    #         right_point = right_pts[i]

    #         # Calculate left angle (from bottom of screen)
    #         left_dy = float(left_point[1] - last_left_point[1])
    #         left_dx = float(left_point[0] - last_left_point[0])
            
    #         # Prevent division by zero error in atan2 by adding small epsilon if both are 0
    #         if left_dy == 0 and left_dx == 0:
    #             left_angle = 0.0
    #         else:
    #             # Subtract result from 180 because we want the angle left side of the line
    #             left_angle = 180.0 - math.degrees(math.atan2(left_dy, left_dx))

    #         # Calculate right angle
    #         right_dy = float(right_point[1] - last_right_point[1])
    #         right_dx = float(right_point[0] - last_right_point[0])
            
    #         if right_dy == 0 and right_dx == 0:
    #             right_angle = 0.0
    #         else:
    #             right_angle = math.degrees(math.atan2(right_dy, right_dx))

    #         # Calculate percent diff of angles
    #         avg_angle = (left_angle + right_angle) / 2.0
    #         if avg_angle == 0:
    #             per_diff_angle = 0.0
    #         else:
    #             per_diff_angle = abs((left_angle - right_angle) / avg_angle)

    #         # Confidence curve: 1 - x^4, where x is percent difference
    #         confidence = max(0.0, 1.0 - (per_diff_angle ** 4))
            
    #         history_conf.append(confidence)
    #         lowest_conf = min(confidence, lowest_conf)

    #         # Update last points
    #         last_left_point = left_point
    #         last_right_point = right_point

    #     if not history_conf:
    #         return 0.0

    #     # Calculate median of the confidences
    #     median_conf = float(np.median(history_conf))

    #     # Final score is weighted: 70% median, 30% lowest confidence found
    #     final_conf = (0.7 * median_conf) + (0.3 * lowest_conf)

    #     self.logger.debug(f"Symmetrical Confidence: {final_conf}")
    #     self.sym_conf = final_conf
    #     return final_conf

    def cal_symmetrical_conf(self, bev_left_lane, bev_right_lane) -> float:
        """
        Calculates symmetrical confidence based on left and right lane geometry in BEV.
        """
        if bev_left_lane is None or bev_right_lane is None or len(bev_left_lane) == 0 or len(bev_right_lane) == 0:
            return 0.0

        left_pts = np.squeeze(bev_left_lane)
        right_pts = np.squeeze(bev_right_lane)
        
        if left_pts.ndim == 1: left_pts = np.array([left_pts])
        if right_pts.ndim == 1: right_pts = np.array([right_pts])

        if len(left_pts) < 2 or len(right_pts) < 2:
            return 0.0

        # Calculate overall angle of the left lane 
        left_dy = float(left_pts[-1][1] - left_pts[0][1])
        left_dx = float(left_pts[-1][0] - left_pts[0][0])
        
        if left_dy == 0 and left_dx == 0:
            left_angle = 0.0
        else:
            left_angle = 180.0 - math.degrees(math.atan2(left_dy, left_dx))

        # Calculate overall angle of the right lane
        right_dy = float(right_pts[-1][1] - right_pts[0][1])
        right_dx = float(right_pts[-1][0] - right_pts[0][0])
        
        if right_dy == 0 and right_dx == 0:
            right_angle = 0.0
        else:
            right_angle = math.degrees(math.atan2(right_dy, right_dx))

        # Calculate percent diff of the overall angles
        avg_angle = (left_angle + right_angle) / 2.0
        if avg_angle == 0:
            per_diff_angle = 0.0
        else:
            per_diff_angle = abs((left_angle - right_angle) / avg_angle)

        # Confidence curve: 1 - x^4, where x is percent difference
        confidence = max(0.0, 1.0 - (per_diff_angle ** 4))
        
        self.logger.debug(f"Symmetrical Confidence: {confidence}")
        self.sym_conf = confidence
        return confidence
    
    # Getters
    def get_pos_conf(self) -> float: return self.pos_conf
    def get_sym_conf(self) -> float: return self.sym_conf