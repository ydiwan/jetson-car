import sys
import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsView, QGraphicsScene, QVBoxLayout, QWidget, QPushButton, QHBoxLayout, QLabel, QStatusBar
from PyQt5.QtGui import QPixmap, QPainter, QBrush, QPen, QColor, QFont
from PyQt5.QtCore import Qt, QPointF, pyqtSignal

# import local modules
from .grid_manager import create_obstacle_grid, visualize_grid, dilate_obstacles
from .path_planner import AStar

class MapView(QGraphicsView):
    point_selected = pyqtSignal(float, float)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        
        # Track selected points and paths
        self.waypoints = []
        self.path_points = []
        self.path_lines = []
        
        # Placeholder for map image and path planner
        self.map_pixmap = None
        self.path_planner = None
        self.obstacle_grid = None
        
        # Grid dimensions
        self.grid_width = 100
        self.grid_height = 100

        self.vicon_origin_x = 0  # Pixel X that corresponds to Vicon (0,0)
        self.vicon_origin_y = 0  # Pixel Y that corresponds to Vicon (0,0)
        self.vicon_scale_x = 1.0  # Scale factor for X axis
        self.vicon_scale_y = 1.0  # Scale factor for Y axis
        self.flip_x = True  # Whether to flip X axis
        self.flip_y = True  # Whether to flip Y axis

    def pixel_to_vicon(self, pixel_x, pixel_y):
        """Convert pixel coordinates to Vicon coordinates"""
        # Calculate relative position from origin
        rel_x = pixel_x - self.vicon_origin_x
        rel_y = pixel_y - self.vicon_origin_y
        
        # Apply flipping if needed
        if self.flip_x:
            rel_x = -rel_x
        if self.flip_y:
            rel_y = -rel_y
        
        # Apply scaling
        vicon_x = rel_x * self.vicon_scale_x
        vicon_y = rel_y * self.vicon_scale_y

        # Apply scaling
        scaled_x = rel_x * self.vicon_scale_x
        scaled_y = rel_y * self.vicon_scale_y
        
        # SWAP X AND Y for Vicon system:
        # - Return y coordinate as vicon_x (vertical axis)
        # - Return x coordinate as vicon_y (horizontal axis)
        vicon_x = scaled_y  # Vicon X = Pixel Y (vertical)
        vicon_y = scaled_x  # Vicon Y = Pixel X (horizontal)
        
        return vicon_x, vicon_y
            
    def load_map(self, map_path):
        """Load the map image and set up the scene"""
        print(f"Attempting to load map from: {map_path}")
        if not os.path.exists(map_path):
            print(f"Error: Map file not found at {map_path}")
            return False

        print(f"Loading map file...")    
        self.map_pixmap = QPixmap(map_path)
        print(f"Map loaded, size: {self.map_pixmap.width()}x{self.map_pixmap.height()}")
        if self.map_pixmap.isNull():
            print("Error: QPixmap is null! Image might be corrupted or the wrong format.")
            return False

        self.map_item = self.scene.addPixmap(self.map_pixmap)
        
        # Adjust view to show the entire map
        self.fitInView(self.map_item, Qt.KeepAspectRatio)
        
        # Scale factors to convert between pixels and real-world coordinates
        self.scale_factor_x = 5588.0 / self.map_pixmap.width()
        self.scale_factor_y = 6096.0 / self.map_pixmap.height()

        # HARDCODED VALUES: Setting Vicon origin to bottom right curve
        self.vicon_origin_x = 962  # Replace with your exact pixel X
        self.vicon_origin_y = 1059  # Replace with your exact pixel Y
        self.vicon_scale_x = self.scale_factor_x  # Use the same scale as normal coordinates
        self.vicon_scale_y = self.scale_factor_y  # Use the same scale as normal coordinates
        self.flip_x = True  # Set to True if Vicon X increases to the left
        self.flip_y = True  # Set to True if Vicon Y increases downward
        
        print(f"Hardcoded Vicon origin set to ({self.vicon_origin_x}, {self.vicon_origin_y})")
        print(f"Map loaded successfully: {map_path}")

        return True
        
    def set_obstacle_grid(self, grid, grid_width, grid_height):
        """Set the obstacle grid for path planning"""
        self.obstacle_grid = grid
        self.grid_width = grid_width
        self.grid_height = grid_height
        
        # Initialize path planner
        self.path_planner = AStar(grid_width, grid_height, grid)
        
    def mousePressEvent(self, event):
        if not self.map_pixmap or not self.path_planner:
            return
            
        if event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            
            # Make sure coordinates are within the map
            if scene_pos.x() < 0 or scene_pos.x() >= self.map_pixmap.width() or \
            scene_pos.y() < 0 or scene_pos.y() >= self.map_pixmap.height():
                return
            
            # Check if clicking on traversable terrain
            grid_x = int((scene_pos.x() / self.map_pixmap.width()) * self.grid_width)
            grid_y = int((scene_pos.y() / self.map_pixmap.height()) * self.grid_height)
            
            if grid_x >= self.grid_width:
                grid_x = self.grid_width - 1
            if grid_y >= self.grid_height:
                grid_y = self.grid_height - 1
                
            if self.obstacle_grid[grid_y][grid_x]:
                print(f"Cannot place waypoint on obstacle at grid ({grid_x}, {grid_y})")
                return
            
            # Convert to Vicon coordinates
            vicon_x, vicon_y = self.pixel_to_vicon(scene_pos.x(), scene_pos.y())
            
            # Print waypoint coordinates to terminal
            print(f"WAYPOINT_ADDED: Pixel=({scene_pos.x():.2f}, {scene_pos.y():.2f}), Vicon=({vicon_x:.2f}, {vicon_y:.2f})")
            
            # Add waypoint marker
            self.add_waypoint_marker(scene_pos.x(), scene_pos.y())
            
            # Emit signal with Vicon coordinates (not pixel coordinates)
            self.point_selected.emit(vicon_x, vicon_y)
            
            # Store waypoint (Vicon coords and pixel coords)
            self.waypoints.append((vicon_x, vicon_y, scene_pos.x(), scene_pos.y()))
            
            # Plan and draw path if we have at least two waypoints
            #if len(self.waypoints) >= 2:
                #self.plan_path(self.waypoints[-2][2:], self.waypoints[-1][2:])
            
    def add_waypoint_marker(self, x, y):
        """Add a visual marker for a waypoint"""
        # Red circle for waypoint
        ellipse = self.scene.addEllipse(x-5, y-5, 10, 10, 
                                       QPen(QColor(255, 0, 0), 2), 
                                       QBrush(QColor(255, 0, 0, 100)))
        
        # Add number label
        text = self.scene.addText(str(len(self.waypoints) + 1))
        text.setFont(QFont("Arial", 10))
        text.setPos(x + 5, y - 15)
        text.setDefaultTextColor(QColor(255, 0, 0))
        
    def plan_path(self, start_point, end_point):
        """Plan a path between two points using A*"""
        # Convert pixel coordinates to grid coordinates
        start_grid_x = int((start_point[0] / self.map_pixmap.width()) * self.grid_width)
        start_grid_y = int((start_point[1] / self.map_pixmap.height()) * self.grid_height)
        
        end_grid_x = int((end_point[0] / self.map_pixmap.width()) * self.grid_width)
        end_grid_y = int((end_point[1] / self.map_pixmap.height()) * self.grid_height)
        
        # Ensure within grid bounds
        start_grid_x = max(0, min(start_grid_x, self.grid_width - 1))
        start_grid_y = max(0, min(start_grid_y, self.grid_height - 1))
        end_grid_x = max(0, min(end_grid_x, self.grid_width - 1))
        end_grid_y = max(0, min(end_grid_y, self.grid_height - 1))
        
        #print(f"Planning path from grid ({start_grid_x}, {start_grid_y}) to grid ({end_grid_x}, {end_grid_y})")
        
        # Find path using A*
        grid_path = self.path_planner.find_path((start_grid_x, start_grid_y), (end_grid_x, end_grid_y))
        
        if grid_path:
            print(f"Found path with {len(grid_path)} grid points")
            
            # Convert grid coordinates to pixel coordinates for drawing
            pixel_path = []
            for grid_x, grid_y in grid_path:
                # Calculate center of grid cell in pixel coordinates
                pixel_x = (grid_x + 0.5) * (self.map_pixmap.width() / self.grid_width)
                pixel_y = (grid_y + 0.5) * (self.map_pixmap.height() / self.grid_height)
                pixel_path.append((pixel_x, pixel_y))
            
            # Clear previous path lines
            for line in self.path_lines:
                self.scene.removeItem(line)
            self.path_lines = []
            
            # Draw path lines between consecutive points
            for i in range(len(pixel_path) - 1):
                line = self.scene.addLine(
                    pixel_path[i][0], pixel_path[i][1],
                    pixel_path[i+1][0], pixel_path[i+1][1],
                    QPen(QColor(0, 0, 255), 2, Qt.DashLine)
                )
                self.path_lines.append(line)
            
            # Store Vicon coordinates for the path
            vicon_path = []
            for pixel_x, pixel_y in pixel_path:
                # Convert each pixel coordinate to Vicon coordinate
                vicon_x, vicon_y = self.pixel_to_vicon(pixel_x, pixel_y)
                vicon_path.append((vicon_x, vicon_y))
            
            # Store the complete path for publishing
            self.path_points = vicon_path
            
            return vicon_path
        else:
            print(f"No path found between grid ({start_grid_x}, {start_grid_y}) and grid ({end_grid_x}, {end_grid_y})")
            return None
        
    def clear_waypoints(self):
        """Clear all waypoints and paths"""
        # Clear the scene but keep the map
        self.scene.clear()
        if self.map_pixmap:
            self.map_item = self.scene.addPixmap(self.map_pixmap)
        
        # Reset waypoints and paths
        self.waypoints = []
        self.path_points = []
        self.path_lines = []
        
    def resizeEvent(self, event):
        """Handle window resize by scaling the view"""
        if self.map_pixmap:
            self.fitInView(self.map_item, Qt.KeepAspectRatio)
        super().resizeEvent(event)


class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('waypoint_gui_node')
        self.waypoints_publisher = self.create_publisher(PoseArray, 'waypoints', 10)
        self.waypoint_names_publisher = self.create_publisher(String, 'waypoint_names', 10)
        
    def publish_waypoints(self, waypoints):
        """Publish waypoints to ROS 2"""
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        names = []
        
        for i, (x, y) in enumerate(waypoints):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            
            # Set identity quaternion
            pose.orientation.w = 1.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            
            pose_array.poses.append(pose)
            names.append(f"Point {i+1}")
        
        # Publish waypoints
        self.waypoints_publisher.publish(pose_array)
        
        # Publish names
        names_msg = String()
        names_msg.data = ",".join(names)
        self.waypoint_names_publisher.publish(names_msg)
        
        self.get_logger().info(f'Published {len(waypoints)} waypoints')


class WaypointSelectionApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cyber City Waypoint Selection")
        
        # Initialize ROS 2
        rclpy.init(args=None)
        self.ros_node = ROS2Publisher()
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Create a label for instructions
        instructions = QLabel("Click on the road to add waypoints. The vehicle will follow the path.")
        instructions.setAlignment(Qt.AlignCenter)
        layout.addWidget(instructions)
        
        # Create map view
        self.map_view = MapView()
        layout.addWidget(self.map_view)
        
        # Create buttons
        button_layout = QHBoxLayout()
        
        self.send_button = QPushButton("Send Waypoints")
        self.send_button.clicked.connect(self.send_waypoints)
        button_layout.addWidget(self.send_button)
        
        self.clear_button = QPushButton("Clear Waypoints")
        self.clear_button.clicked.connect(self.map_view.clear_waypoints)
        button_layout.addWidget(self.clear_button)
        
        layout.addLayout(button_layout)
        
        # Create status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        
        # Connect signals
        self.map_view.point_selected.connect(self.on_point_selected)
        
        # Set window size
        self.resize(800, 600)
        
        # Load map and obstacle grid
        self.load_map_and_grid()
        
    def load_map_and_grid(self):
        """Load the map and create the obstacle grid"""
        # Try multiple possible locations for the map
        possible_map_locations = [
            # Direct path (if you specified it)
            "/path/to/your/cyber_city_map.png",
            
            # In package directory
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'maps', 'cyber_city_map.png'),
            
            # In install directory
            os.path.join(os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0], 'share', 'vehicle_gui', 'maps', 'cyber_city_map.png'),
            
            # In home directory (if you copied it there)
            os.path.join(os.path.expanduser('~'), 'cyber_city_map.png')
        ]
        
        # Try each location
        map_path = None
        for path in possible_map_locations:
            if os.path.exists(path):
                map_path = path
                print(f"Found map at: {map_path}")
                break
                
        if not map_path:
            self.statusBar.showMessage("Map not found. Please check console for attempted paths.")
            print("Attempted to find map at these locations:")
            for path in possible_map_locations:
                print(f" - {path}")
            return
                
        # Load the map
        if not self.map_view.load_map(map_path):
            self.statusBar.showMessage("Failed to load map.")
            return
                
        # Create obstacle grid
        grid_width = 100
        grid_height = 100
        obstacle_grid = create_obstacle_grid(map_path, grid_width, grid_height)
        
        # Set the grid in the map view
        self.map_view.set_obstacle_grid(obstacle_grid, grid_width, grid_height)
        
        self.statusBar.showMessage("Map and obstacle grid loaded successfully.")
        
    def on_point_selected(self, x, y):
        """Handle point selection"""
        point_count = len(self.map_view.waypoints)
        self.statusBar.showMessage(f"Added waypoint {point_count} at ({x:.1f}, {y:.1f})")
        
    def send_waypoints(self):
        """Send waypoints to the vehicle"""
        if self.map_view.waypoints:
            # Send waypoints directly (no pathfinding)
            waypoint_coords = [(wp[0], wp[1]) for wp in self.map_view.waypoints]
            
            for i, (x, y) in enumerate(waypoint_coords):
                print(f"Waypoint {i+1}: Vicon=(X(vert)={x:.2f}, Y(horiz)={y:.2f})")
                
            self.ros_node.publish_waypoints(waypoint_coords)
            self.statusBar.showMessage(f"Published {len(waypoint_coords)} waypoints")
        else:
            self.statusBar.showMessage("No waypoints to send")
        # Extract just the real-world coordinates (x, y) from the paths
        # if self.map_view.path_points:
        #     self.ros_node.publish_waypoints(self.map_view.path_points)
        #     self.statusBar.showMessage(f"Published {len(self.map_view.path_points)} path points")
        # elif self.map_view.waypoints:
        #     # If we have waypoints but no path (should not happen), just send waypoints
        #     real_waypoints = [(wp[0], wp[1]) for wp in self.map_view.waypoints]
        #     self.ros_node.publish_waypoints(real_waypoints)
        #     self.statusBar.showMessage(f"Published {len(real_waypoints)} waypoints")
        # else:
        #     self.statusBar.showMessage("No waypoints to send")
        
    def closeEvent(self, event):
        """Clean up ROS 2 on exit"""
        rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = WaypointSelectionApp()
    window.show()
    
    # Spin ROS 2 in a separate thread
    import threading
    def spin_ros():
        while rclpy.ok():
            rclpy.spin_once(window.ros_node, timeout_sec=0.1)
    threading.Thread(target=spin_ros, daemon=True).start()
    
    return app.exec_()


if __name__ == "__main__":
    main()