import rclpy
import numpy as np
from .ld_config import LdConfig 

class LdScanner:
    def __init__(self, config: LdConfig, logger=None):
        self.config = config
        
        # Lane Data Storage
        self.left_lane = []
        self.right_lane = []
        self.center_lane = []
        
        self.known_lane_width = 415 
        
        # Sliding Window Parameters
        self.margin = 60  # Width of the window (+/- margin)
        self.minpix = 5   # Min pixels needed to recenter the window
        
        # ROS2 Logger
        self.logger = logger or rclpy.logging.get_logger('Ld_scanner')

    def run(self, bin_mask: np.ndarray) -> bool:
        self.left_lane.clear()
        self.right_lane.clear()
        self.center_lane.clear()

        rows, cols = bin_mask.shape
        self.margin = 120  

        # Histogram on the bottom half
        bottom_half = bin_mask[rows//2:, :]
        histogram = np.sum(bottom_half, axis=0)

        midpoint = cols // 2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        
        # Rigid Fallback for the starting bases
        if histogram[leftx_base] < 10 and histogram[rightx_base] > 10:
            leftx_base = rightx_base - self.known_lane_width
        elif histogram[rightx_base] < 10 and histogram[leftx_base] > 10:
            rightx_base = leftx_base + self.known_lane_width
        elif histogram[leftx_base] < 10 and histogram[rightx_base] < 10:
            return False 

        # Force the bases to perfectly match the known physical width
        current_base_width = rightx_base - leftx_base
        if abs(current_base_width - self.known_lane_width) > 50: 
            if histogram[leftx_base] > histogram[rightx_base]:
                rightx_base = leftx_base + self.known_lane_width # Force right into position
            else:
                leftx_base = rightx_base - self.known_lane_width # Force left into position

        leftx_current = leftx_base
        rightx_current = rightx_base

        # Slide the windows
        y_steps = range(rows - 1, 0, -self.config.steps)
        
        for y in y_steps:
            win_xleft_low = max(0, leftx_current - self.margin)
            win_xleft_high = min(cols, leftx_current + self.margin)
            win_xright_low = max(0, rightx_current - self.margin)
            win_xright_high = min(cols, rightx_current + self.margin)

            row = bin_mask[y, :]

            good_left_inds = np.nonzero(row[win_xleft_low:win_xleft_high])[0] + win_xleft_low
            good_right_inds = np.nonzero(row[win_xright_low:win_xright_high])[0] + win_xright_low

            found_left = len(good_left_inds) > self.minpix
            found_right = len(good_right_inds) > self.minpix

            if found_left:
                leftx_current = int(np.mean(good_left_inds))
            if found_right:
                rightx_current = int(np.mean(good_right_inds))

            # Never update known_lane_width. If one line vanishes, rigidly project the other.
            if found_left and not found_right:
                rightx_current = leftx_current + self.known_lane_width
            elif found_right and not found_left:
                leftx_current = rightx_current - self.known_lane_width
            elif found_left and found_right:
                # Even if both are found, enforce the rigid structure to prevent collapse
                actual_width = rightx_current - leftx_current
                if abs(actual_width - self.known_lane_width) > 50:
                    if len(good_left_inds) > len(good_right_inds):
                        rightx_current = leftx_current + self.known_lane_width
                    else:
                        leftx_current = rightx_current - self.known_lane_width

            center_x = int((leftx_current + rightx_current) / 2)
            
            self.left_lane.append((leftx_current, y))
            self.right_lane.append((rightx_current, y))
            self.center_lane.append((center_x, y))

        self.left_lane.reverse()
        self.right_lane.reverse()
        self.center_lane.reverse()

        self.median_filter()

        return len(self.center_lane) > 3

    def median_filter(self):
        """ Sliding window median filter on the scan results """
        if not self.center_lane:
            return

        filtered_center, filtered_left, filtered_right = [], [], []
        window_size = self.config.median_window
        threshold = self.config.median_threshold
        n = len(self.center_lane)
        
        for i in range(n):
            start = max(0, i - window_size // 2)
            end = min(n - 1, i + window_size // 2)
            window_values = [self.center_lane[j][0] for j in range(start, end + 1) if j != i]
            
            if window_values:
                median_val = np.median(window_values)
                if abs(self.center_lane[i][0] - median_val) <= threshold:
                    filtered_center.append(self.center_lane[i])
                    filtered_left.append(self.left_lane[i])
                    filtered_right.append(self.right_lane[i])
            else:
                filtered_center.append(self.center_lane[i])
                filtered_left.append(self.left_lane[i])
                filtered_right.append(self.right_lane[i])

        if filtered_center:
            self.left_lane, self.right_lane, self.center_lane = filtered_left, filtered_right, filtered_center
        else:
            self.logger.debug("The scanner found no valid lanes after filtering")

    # Getters
    def get_left_lane(self): return self.left_lane
    def get_right_lane(self): return self.right_lane
    def get_center_lane(self): return self.center_lane