import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    video_height = 720
    video_width = 1280
    
    return LaunchDescription([ 
        # 1. Hardware Motor Control (Back Wheels)
        Node(
            package='gpio_node',
            executable='gpio_node',
            name='gpio_node',
            output='screen'
        ),
        
        # 2. Teleop Bridge (Translates Twist to Maestro/GPIO)
        Node(
            package='vehicle_controller_py',
            executable='teleop_bridge',
            name='teleop_bridge',
            output='screen'
        ),

        # 3. Web Video Server (To view the camera feed on your laptop)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),

        # 4. Vision Pipeline (Optional, but lets you see the lane detection while driving)
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
    ])