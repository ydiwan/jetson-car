import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    video_height = 720
    video_width = 1280
    
    return LaunchDescription([ 
        # Hardware Motor Control (Back Wheels)
        Node(
            package='gpio_node',
            executable='gpio_node',
            name='gpio_node',
            output='screen'
        ),
        
        # Teleop Bridge (Translates Twist to Maestro/GPIO)
        Node(
            package='vehicle_controller_py',
            executable='teleop_bridge',
            name='teleop_bridge',
            output='screen'
        ),

        # Web Video Server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),
        
        # Traffic Light Detection
        Node(
            package='traffic_light_py',
            executable='traffic_light_node',
            name='traffic_light_node',
            output='screen'
        ),
    ])