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

from .grid_manager import create_obstacle_grid, visualize_grid, dilate_obstacles
from .path_planner import DijkstraClass

class MapView(QGraphicsView):
    point_selected = pyqtSignal(float, float)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        
        self.waypoints = []
        self.path_points = []
        self.path_lines = []
        
        self.map_pixmap = None
        self.path_planner = None
        #self.obstacle_grid = None
        
        self.node_markers = []

        self.vicon_origin_x = 0  # Pixel X that corresponds to Vicon (0,0)
        self.vicon_origin_y = 0  # Pixel Y that corresponds to Vicon (0,0)
        self.vicon_scale_x = 1.0  # Scale factor for X axis
        self.vicon_scale_y = 1.0  # Scale factor for Y axis

    def pixel_to_vicon(self, pixel_x, pixel_y):
        rel_x = pixel_x - self.vicon_origin_x
        rel_y = pixel_y - self.vicon_origin_y
        
        # flip axes
        rel_x = -rel_x
        rel_y = -rel_y
        
        scaled_x = rel_x * self.vicon_scale_x
        scaled_y = rel_y * self.vicon_scale_y
        
        # swap x and y axes for Vicon system
        vicon_x = scaled_y
        vicon_y = scaled_x
        
        return vicon_x, vicon_y
    
    def vicon_to_pixel(self, vicon_x, vicon_y):
        scaled_y = vicon_x
        scaled_x = vicon_y
        
        # unscale
        rel_y = scaled_y / self.vicon_scale_y
        rel_x = scaled_x / self.vicon_scale_x
        
        # flip
        rel_y = -rel_y
        rel_x = -rel_x
        
        pixel_x = rel_x + self.vicon_origin_x
        pixel_y = rel_y + self.vicon_origin_y
        
        return pixel_x, pixel_y

    def load_map(self, map_path):
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
        
        self.fitInView(self.map_item, Qt.KeepAspectRatio)
        
        # scale factors to convert between pixels and real-world coordinates
        self.scale_factor_x = 5588.0 / self.map_pixmap.width()
        self.scale_factor_y = 6096.0 / self.map_pixmap.height()

        # setting Vicon origin to bottom right curve
        self.vicon_origin_x = 962 
        self.vicon_origin_y = 1059
        self.vicon_scale_x = self.scale_factor_x
        self.vicon_scale_y = self.scale_factor_y

        print(f"Map loaded successfully: {map_path}")

        # initialize Dijkstra planner
        try:
            self.path_planner = DijkstraClass('nodes.csv', 'edges.csv')
            self.visualize_nodes()
            print("Dijkstra path planner initialized")
        except FileNotFoundError as e:
            print(f"Error loading graph files: {e}")

        return True
        
    def visualize_nodes(self):
        if not self.path_planner:
            return

        for node_id, node_data in self.path_planner.nodes.items():
            if '_' not in node_id:
                pixel_x, pixel_y = self.vicon_to_pixel(node_data['x'], node_data['y'])

                # draw circle
                circle = self.scene.addEllipse(
                    pixel_x - 40, pixel_y - 40, 80, 80, # -x for offset (centers circle on node), width, height (same for circle)
                    QPen(QColor(0, 255, 0), 2),
                    QBrush(QColor(0, 255, 0, 50))
                )

                # label nodes
                text = self.scene.addText(node_id)
                text.setFont(QFont("Arial", 30)) # font, size
                text.setPos(pixel_x - 30, pixel_y - 30)
                text.setDefaultTextColor(QColor(255, 255, 255))
                
                self.node_markers.append(circle)
                self.node_markers.append(text)
            else: # for intermediate nodes
                pixel_x, pixel_y = self.vicon_to_pixel(node_data['x'], node_data['y'])
                
                # draw smaller circle
                circle = self.scene.addEllipse(
                    pixel_x - 10, pixel_y - 10, 20, 20, # -x for offset (centers circle on node), width, height (same for circle)
                    QPen(QColor(0, 255, 0), 2),
                    QBrush(QColor(0, 255, 0, 50))
                )
        
    def mousePressEvent(self, event):
        if not self.map_pixmap or not self.path_planner:
            return
            
        if event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            
            # make sure coordinates are within the map
            if scene_pos.x() < 0 or scene_pos.x() >= self.map_pixmap.width() or \
                scene_pos.y() < 0 or scene_pos.y() >= self.map_pixmap.height():
                return
            
            click_x, click_y = self.pixel_to_vicon(scene_pos.x(), scene_pos.y())

            nearest_node, distance = self.path_planner.find_nearest_node(click_x, click_y)

            # snap to node
            if distance < 500:
                node_x, node_y = self.path_planner.get_node_position(nearest_node)
                pixel_x, pixel_y = self.vicon_to_pixel(node_x, node_y)
                scene_pos = QPointF(pixel_x, pixel_y)

                # convert to Vicon coordinates
                vicon_x = node_x
                vicon_y = node_y
            
            else:
                vicon_x = click_x
                vicon_y = click_y

            print(f"WAYPOINT_ADDED: Pixel=({scene_pos.x():.2f}, {scene_pos.y():.2f}), Vicon=({vicon_x:.2f}, {vicon_y:.2f})")
            
            # add waypoint marker
            self.add_waypoint_marker(scene_pos.x(), scene_pos.y())
            
            # emit signal
            self.point_selected.emit(vicon_x, vicon_y)
            
            # store waypoint
            self.waypoints.append((vicon_x, vicon_y, vicon_x, vicon_y, scene_pos.x(), scene_pos.y()))
            
            # plan path
            if len(self.waypoints) >= 2:
                # Use real coordinates for path planning
                start_real = (self.waypoints[-2][2], self.waypoints[-2][3])
                end_real = (self.waypoints[-1][2], self.waypoints[-1][3])
                self.plan_path(start_real, end_real)
            
    def add_waypoint_marker(self, x, y):
        ellipse = self.scene.addEllipse(x-5, y-5, 10, 10, 
                                       QPen(QColor(255, 0, 0), 2), 
                                       QBrush(QColor(255, 0, 0, 100)))
        
        # add number label
        text = self.scene.addText(str(len(self.waypoints) + 1))
        text.setFont(QFont("Arial", 10))
        text.setPos(x + 5, y - 15)
        text.setDefaultTextColor(QColor(255, 0, 0))
        
    def plan_path(self, start_point, end_point):
        if not self.path_planner:
            print("Dijkstra planner not initialized")
            return None
            
        print(f"Planning path from ({start_point[0]:.2f}, {start_point[1]:.2f}) to ({end_point[0]:.2f}, {end_point[1]:.2f})")
        
        # find path
        path_nodes, path_coords, total_distance = self.path_planner.find_path_from_coordinates(
            start_point[0], start_point[1],
            end_point[0], end_point[1]
        )
        
        if path_nodes:
            print(f"Found path through nodes: {' -> '.join(path_nodes)}")
            print(f"Total distance: {total_distance:.2f}")
            
            # Clear previous path lines
            for line in self.path_lines:
                self.scene.removeItem(line)
            self.path_lines = []
            
            # Convert path coordinates to pixels and draw
            pixel_path = []
            for vicon_x, vicon_y in path_coords:
                pixel_x, pixel_y = self.vicon_to_pixel(vicon_x, vicon_y)
                pixel_path.append((pixel_x, pixel_y))
            
            # Draw path lines
            for i in range(len(pixel_path) - 1):
                line = self.scene.addLine(
                    pixel_path[i][0], pixel_path[i][1],
                    pixel_path[i+1][0], pixel_path[i+1][1],
                    QPen(QColor(0, 0, 255), 3)
                )
                self.path_lines.append(line)
        
            self.path_points = path_coords
            return path_coords
        else:
            print("No path found!")
            return None
        
    def clear_waypoints(self):
        # Clear the scene but keep the map and nodes
        for item in self.scene.items():
            if item != self.map_item and item not in self.node_markers:
                self.scene.removeItem(item)
        
        # Reset waypoints and paths
        self.waypoints = []
        self.path_points = []
        self.path_lines = []
        
    def resizeEvent(self, event):
        if self.map_pixmap:
            self.fitInView(self.map_item, Qt.KeepAspectRatio)
        super().resizeEvent(event)


class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('waypoint_gui_node')
        self.waypoints_publisher = self.create_publisher(PoseArray, 'waypoints', 10)
        
    def publish_waypoints(self, waypoints):
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
        
        self.statusBar.showMessage("Map loaded successfully.")
        
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