import rclpy
import numpy as np
from .ld_config import LdConfig # Assumes ld_config is available in the same folder

class LdScanner:
    def __init__(self, config: LdConfig, logger=None):
        self.config = config
        
        # Lane Data Storage - storing as (x, y) tuples
        self.left_lane = []
        self.right_lane = []
        self.center_lane = []
        
        # ROS2 Logger
        self.logger = logger or rclpy.logging.get_logger('Ld_scanner')

    def run(self, bin_mask: np.ndarray) -> bool:
        """
        Scans binary images for lane line segments based on width and position.
        Returns True if more than 3 lane points are found.
        """
        # Reset the lane pixel coordinate vectors
        self.left_lane.clear()
        self.right_lane.clear()
        self.center_lane.clear()

        rows, cols = bin_mask.shape

        # Scan rows based on the configured step size
        for y in range(0, rows, self.config.steps):
            lane_segments = []
            
            # Extract the whole row and find indices of all white pixels (>0)
            row = bin_mask[y, :]
            nonzero_indices = np.nonzero(row)[0]
            
            if len(nonzero_indices) > 0:
                # Find gaps greater than 1 to separate contiguous line segments
                split_points = np.where(np.diff(nonzero_indices) > 1)[0] + 1
                segments = np.split(nonzero_indices, split_points)

                for seg in segments:
                    if len(lane_segments) == 2:
                        break # We only need the left and right lane
                    
                    segment_start = seg[0]
                    segment_end = seg[-1]
                    lane_width = segment_end - segment_start
                    
                    # Check if the segment width matches what a lane should look like
                    if self.config.min_lane_width <= lane_width <= self.config.max_lane_width:
                        lane_segments.append((segment_start, segment_end))
                        self.logger.debug(f"Add Lane Segment that is {lane_width} pixels wide")
            
            # Process lane segments if we found at least 2 (left and right)
            if len(lane_segments) >= 2:
                left_lane_right_edge = lane_segments[0][1]
                right_lane_left_edge = lane_segments[-1][0]
                
                road_width = abs(right_lane_left_edge - left_lane_right_edge)
                self.logger.debug(f"Road width: {road_width}")
                
                # Check if the road width makes sense
                if self.config.min_road_width <= road_width <= self.config.max_road_width:
                    
                    # Find the midpoint between the left and right lane
                    center_x = (left_lane_right_edge + right_lane_left_edge) // 2
                    
                    # Store the points as (x, y) tuples
                    self.center_lane.append((center_x, y))
                    self.right_lane.append((right_lane_left_edge, y))
                    self.left_lane.append((left_lane_right_edge, y))
                    
                    self.logger.debug(f"Left, center, right: ({left_lane_right_edge},{y}) ({right_lane_left_edge},{y}) ({center_x},{y})")

        # Do a median filter to remove outlier points 
        # (Moved outside the 'y' loop for massive performance gain)
        self.median_filter()

        # Return True if we found enough points
        return len(self.left_lane) > 3

    def median_filter(self):
        """
        Does a sliding window median filter on the left, right, and center lane scan results.
        """
        if not self.center_lane:
            return

        filtered_center = []
        filtered_left = []
        filtered_right = []
        
        window_size = self.config.median_window
        threshold = self.config.median_threshold
        
        n = len(self.center_lane)
        
        # Go through every element
        for i in range(n):
            # Calculate current window start and end index
            start = max(0, i - window_size // 2)
            end = min(n - 1, i + window_size // 2)
            
            # Get x-values in the window, skipping the current element 'i'
            window_values = [self.center_lane[j][0] for j in range(start, end + 1) if j != i]
            
            if window_values:
                # Calculate median
                median_val = np.median(window_values)
                
                # Check if the current index is within the threshold
                if abs(self.center_lane[i][0] - median_val) <= threshold:
                    filtered_center.append(self.center_lane[i])
                    filtered_left.append(self.left_lane[i])
                    filtered_right.append(self.right_lane[i])
            else:
                # Keep first/last points if no neighbors
                filtered_center.append(self.center_lane[i])
                filtered_left.append(self.left_lane[i])
                filtered_right.append(self.right_lane[i])

        if filtered_center:
            # Assign the new filtered values
            self.left_lane = filtered_left
            self.right_lane = filtered_right
            self.center_lane = filtered_center
        else:
            self.logger.debug("The scanner found no valid lanes after filtering")

    # Getters
    def get_left_lane(self): return self.left_lane
    def get_right_lane(self): return self.right_lane
    def get_center_lane(self): return self.center_lane