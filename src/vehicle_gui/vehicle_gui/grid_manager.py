import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import os

def create_obstacle_grid(image_path, grid_width, grid_height, road_threshold=100):
    """
    Create a grid of obstacles where:
    - Roads (black/dark gray) are traversable, even with some glare
    - Yellow lines are obstacles (to enforce lane discipline)
    - Everything else (buildings, grass, etc.) are obstacles
    
    Parameters:
    - image_path: Path to the map image
    - grid_width, grid_height: Dimensions of the grid
    - road_threshold: Maximum RGB value to be considered road (higher = more lenient)
    
    Returns a grid where True = obstacle, False = traversable
    """
    # Load image
    original_img = Image.open(image_path)
    original_img = original_img.convert("RGB")
    
    # Resize for grid
    img = original_img.resize((grid_width, grid_height))
    
    # Convert to numpy array
    pixels = np.array(img)
    
    # Create a grayscale version for more robust road detection
    gray_pixels = np.mean(pixels, axis=2)
    
    # Create mask for road pixels (traversable)
    # Now using a higher threshold to capture areas with glare
    road_mask = (gray_pixels < road_threshold)
    
    # Create mask for yellow lines (obstacles to enforce lane discipline)
    # Yellow has high R and G, low B
    yellow_line_mask = (pixels[:,:,0] > 180) & (pixels[:,:,1] > 180) & (pixels[:,:,2] < 100)
    
    # Create mask for white lines (optional, if needed)
    white_line_mask = (pixels[:,:,0] > 200) & (pixels[:,:,1] > 200) & (pixels[:,:,2] > 200)
    
    # Black roads are traversable (False), everything else is an obstacle (True)
    # 1. Start with everything as an obstacle
    obstacle_grid = np.ones((grid_height, grid_width), dtype=bool)
    
    # 2. Set road areas as traversable
    obstacle_grid[road_mask] = False
    
    # 3. Set yellow lines as obstacles (to enforce lane keeping)
    obstacle_grid[yellow_line_mask] = True
    
    return obstacle_grid

def visualize_grid(obstacle_grid, output_path="obstacle_grid.png"):
    """Visualize the obstacle grid for debugging"""
    plt.figure(figsize=(10, 10))
    plt.imshow(obstacle_grid, cmap='binary')
    plt.title('Obstacle Grid (White = Obstacle, Black = Traversable Road)')
    plt.savefig(output_path)
    plt.close()
    print(f"Grid visualization saved to {output_path}")

def dilate_obstacles(obstacle_grid, kernel_size=3):
    """
    Expand obstacles slightly to add a safety margin
    This helps prevent the vehicle from planning paths too close to obstacles
    """
    from scipy import ndimage
    
    # Create a kernel for dilation
    kernel = np.ones((kernel_size, kernel_size), dtype=bool)
    
    # Dilate the obstacles
    dilated_grid = ndimage.binary_dilation(obstacle_grid, structure=kernel)
    
    return dilated_grid

# testing
# if __name__ == "__main__":
#     # Test with an example image
#     image_path = "~/ros2_ws/src/vehicle_gui/maps/cyber_city_map.png"
#     if os.path.exists(image_path):
#         grid = create_obstacle_grid(image_path, 100, 100)
#         visualize_grid(grid)
        
#         # Also create a dilated version for safety margins
#         dilated_grid = dilate_obstacles(grid)
#         visualize_grid(dilated_grid, "dilated_grid.png")
#     else:
#         print(f"Test image not found at {image_path}")