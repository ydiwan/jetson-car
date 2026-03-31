import rclpy
import cv2
from dataclasses import dataclass
import numpy as np
from .ld_config import LdConfig
from .ld_preprocessor import LdPreprocessor
from .ld_scanner import LdScanner
from .ld_postprocessor import LdPostprocessor
from .ld_conf_calculator import LdConfCalculator

@dataclass
class LdResults:
    """Struct to hold the lane detection results."""
    ld_img: np.ndarray = None
    bin_mask: np.ndarray = None
    filter_mask: np.ndarray = None
    scanner_mask: np.ndarray = None
    bev: np.ndarray = None
    delta: int = 0
    position_conf: float = 0.0
    symmetry_conf: float = 0.0

class LaneDetector:
    def __init__(self, node):
        self.logger = node.get_logger()
        self.logger.info("Initializing LaneDetector Pipeline...")

        # Initialize Configuration
        self.config = LdConfig(node)

        # Calculate scaled width for downstream modules
        scaled_width = int(self.config.rgb_img_width * self.config.scale_res)
        scaled_center = scaled_width // 2

        self.logger.debug(f"The scaled width of the image: {scaled_width}")

        # Initialize Pipeline Modules
        self.preprocessor = LdPreprocessor(self.config)
        self.scanner = LdScanner(self.config, logger=self.logger)
        self.postprocessor = LdPostprocessor(self.config, scaled_width, logger=self.logger)
        self.conf_calculator = LdConfCalculator(self.config, scaled_center, logger=self.logger)

        # Initialize results container
        self.results = LdResults()

    def step(self, rgb_img: np.ndarray) -> LdResults:
        """
        Runs the frame through the entire pipeline and returns the results.
        """
        # Preprocess the image 
        processed_img = self.preprocessor.run(rgb_img)

        # Save masks to results
        self.results.bin_mask = self.preprocessor.bin_mask
        self.results.filter_mask = self.preprocessor.filter_bin_mask
        
        # Initialize a completely blank black mask for the scanner output
        self.results.scanner_mask = np.zeros_like(self.results.filter_mask)

        # Scan for road lane lines
        lanes_found = self.scanner.run(self.results.filter_mask)

        if lanes_found:
            left_lane = self.scanner.get_left_lane()
            right_lane = self.scanner.get_right_lane()
            center_lane = self.scanner.get_center_lane()

            # Draw only verified liens
            if left_lane is not None and len(left_lane) > 0:
                l_pts = np.array(left_lane, np.int32).reshape((-1, 1, 2))
                cv2.polylines(self.results.scanner_mask, [l_pts], isClosed=False, color=255, thickness=10)

            if right_lane is not None and len(right_lane) > 0:
                r_pts = np.array(right_lane, np.int32).reshape((-1, 1, 2))
                cv2.polylines(self.results.scanner_mask, [r_pts], isClosed=False, color=255, thickness=10)

            # Postprocess (Draw visuals, calculate delta, BEV transform)
            self.results.delta = self.postprocessor.run(
                processed_img, 
                left_lane, 
                right_lane,
                center_lane
            )

            # Store image results
            self.results.ld_img = self.postprocessor.get_ld_result()
            self.results.bev = self.postprocessor.get_bev_img()

            # Calculate Confidences
            target_x = center_lane[-1][0] 
            self.results.position_conf = self.conf_calculator.cal_position_conf(target_x)
            
            self.results.symmetry_conf = self.conf_calculator.cal_symmetrical_conf(
                self.postprocessor.get_bev_left_lane(), 
                self.postprocessor.get_bev_right_lane()
            )
        else:
            # The lane_scanner didn't find anything
            self.results.position_conf = 0.0
            self.results.symmetry_conf = 0.0
            self.results.delta = 0

        return self.results