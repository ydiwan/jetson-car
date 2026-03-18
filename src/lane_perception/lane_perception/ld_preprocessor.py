import cv2
import numpy as np
from .ld_config import LdConfig

class LdPreprocessor:
    def __init__(self, config: LdConfig):
        self.config = config
        self.hsv_img = None
        self.white_mask = None
        self.yellow_mask = None
        self.bin_mask = None
        self.filter_bin_mask = None
        
        # Define a kernel for morphological operations (noise removal)
        self.kernel = np.ones((3, 3), np.uint8) 

    def run(self, rgb_img):
        """Main interface to run the preprocessing pipeline."""
        cropped_resized_img = self.crop_n_resize(rgb_img)
        self.lane_mask(cropped_resized_img)
        
        # Return the manipulated image so the main node can use it
        return cropped_resized_img

    def crop_n_resize(self, rgb_img):
        """Crops the top portion of the image and scales the resolution."""
        height = rgb_img.shape[0]
        start_row = height - int(height / self.config.scale_height)
        
        # Crop the image
        cropped_img = rgb_img[start_row:height, :]
        
        # Scale resolution
        resized_img = cv2.resize(
            cropped_img, 
            (0, 0), # Let fx and fy dictate the size
            fx=self.config.scale_res, 
            fy=self.config.scale_res
        )
        return resized_img

    def lane_mask(self, rgb_img):
        """Generates color masks and filters them."""
        # Convert RGB to HSV
        self.hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)

        # Get White mask
        lower_white = np.array([0, 0, 190])
        upper_white = np.array([180, 60, 255])
        self.white_mask = cv2.inRange(self.hsv_img, lower_white, upper_white)

        # Get Yellow mask
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        self.yellow_mask = cv2.inRange(self.hsv_img, lower_yellow, upper_yellow)

        # Combine the masks
        self.bin_mask = cv2.bitwise_or(self.white_mask, self.yellow_mask)

        # Remove noise using a morphological OPEN operation
        self.bin_mask = cv2.morphologyEx(self.bin_mask, cv2.MORPH_OPEN, self.kernel)

        # Filter the binary mask by contour area
        self.filter_area()

    def filter_area(self):
        """Removes contours that are too small or too large based on config."""
        # Find contours
        contours, _ = cv2.findContours(
            self.bin_mask, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Initialize an empty black image the same size as bin_mask
        self.filter_bin_mask = np.zeros_like(self.bin_mask)

        # Iterate through contours and filter by area
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if self.config.min_area <= area <= self.config.max_area:
                # Draw the valid contour filled in with white (255)
                cv2.drawContours(self.filter_bin_mask, [contour], 0, 255, cv2.FILLED)