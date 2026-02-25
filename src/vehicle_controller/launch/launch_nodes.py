from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    video_height = 720
    video_width = 1280
    
    return LaunchDescription([ 
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),
        Node(
            package='lane_detection',
            executable='lane_detection_node',
            name='lane_detection_node',
            arguments=[
                '--ros-args',
                '--log-level', 'lane_detection_node:=info',
                '--log-level', 'lane_detector:=info',
                '--log-level', 'Ld_postprocessor:=info',
                '--log-level', 'Ld_scanner:=info',
                '--log-level', 'Ld_conf_calculator:=info',
                '--log-level', 'Ld_preprocessor:=info'
            ],
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
                'max_area':2000,
                'min_lane_width': 8,
                'max_lane_width': 20,
                'min_road_width': 200,
                'max_road_width': 500,
                'median_window': 10,
                'median_threshold': 5.0,
                'center_offset': 0.0,
                'position_conf_threshold': 0.1,
                'top_left_src':  [174.0, 0.0],
                'top_right_src': [496.0, 0.0],
                'btm_left_src':  [ 30.0, 159.0],
                'btm_right_src': [627.0, 159.0],
                
            }],
            output='screen'
        ),
        Node(
            package='vehicle_controller',
            executable='waypoint_loader_node',
            name='waypoint_loader',
            parameters=[{
                'csv_file': "src/vehicle_controller/src/coordinates.csv",
                'publish_frequency' : 10.0    
            }],
            output='screen'
        ),
        Node(
            package='vehicle_controller',
            executable='vehicle_controller_node',
            name='vehicle_controller_node',
            parameters=[{
                'vicon_kp'  : 25.0,
                'ld_kp'     : 3.0,
                'servo_min' : 1000,
                'servo_max' : 2000,
                'servo_usb' : "/dev/ttyACM0"
                
                }],
            arguments=[
                '--ros-args', 
                '--log-level', 'vehicle_controller_node:=debug',
                '--log-level', 'vehicle_class:=debug',  
                '--log-level', 'maestro_servo_controller:=debug'  
            ],
            output='screen'
        ),
        Node(
            package='gpio_node',
            executable='gpio_node',
            name='gpio_node',
            arguments=[
                '--ros-args', 
                '--log-level', 'gpio_node:=info'
            ],
            output='screen'
        ),
    ])