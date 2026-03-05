import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Helper to find paths in the install directory
    vehicle_pkg_dir = get_package_share_directory('vehicle_controller_py')
    
    # Path to the coordinates.csv in the installed config folder
    csv_file_path = os.path.join(vehicle_pkg_dir, 'config', 'coordinates.csv')
    
    video_height = 720
    video_width = 1280
    
    return LaunchDescription([ 
        # Standard Web Video Server (C++ Package)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),

        # Lane Detection Node 
        Node(
            package='lane_detection_py',
            executable='lane_detection_node',
            name='lane_detection_node',
            parameters=[{
                'camera_index': 0,
                'width': video_width,
                'height': video_height,
                'fps': 60,
                'scale_height': 3,
                'scale_res': 0.53,
                'input_img_width': video_width,
                'steps': 3,
                'min_area': 100,
                'max_area': 2000,
                'min_lane_width': 8,
                'max_lane_width': 20,
                'min_road_width': 200,
                'max_road_width': 500,
                'median_window': 10,
                'median_threshold': 5.0,
                'center_offset': 0.0,
                'position_conf_threshold': 0.1,
                'top_left_src': [174.0, 0.0],
                'top_right_src': [496.0, 0.0],
                'btm_left_src': [30.0, 159.0],
                'btm_right_src': [627.0, 159.0],
            }],
            output='screen'
        ),

        # Waypoint Loader 
        Node(
            package='vehicle_controller_py',
            executable='waypoint_loader_node',
            name='waypoint_loader',
            parameters=[{
                'csv_file': csv_file_path,
                'publish_frequency': 10.0,
                'frame_id': 'map'
            }],
            output='screen'
        ),

        # Vehicle Controller
        Node(
            package='vehicle_controller_py',
            executable='vehicle_controller_node',
            name='vehicle_controller_node',
            parameters=[{
                'vicon_kp': 25.0,
                'ld_kp': 3.0,
                'servo_min': 1000,
                'servo_max': 2000,
                'servo_usb': "/dev/ttyACM0"
            }],
            arguments=['--ros-args', '--log-level', 'debug'],
            output='screen'
        ),

        # GPIO Node 
        Node(
            package='gpio_node',
            executable='gpio_node',
            name='gpio_node',
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'
        ),
        
        # Speedometer Node 
        Node(
            package='speedometer_py',
            executable='speedometer_node',
            name='speedometer',
            parameters=[{
                'median_window_size': 5,
                'median_threshold': 2.0
            }],
            output='screen'
        ),
    ])