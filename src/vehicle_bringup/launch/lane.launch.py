import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments
    hardware_type = LaunchConfiguration('hardware_type', default='real')
    show_sim = LaunchConfiguration('show_sim', default='false')
    enable_ai = LaunchConfiguration('enable_ai',  default='false')
    enable_drive = LaunchConfiguration('enable_drive', default='true')

    # Path to bringup package
    bringup_dir = get_package_share_directory('vehicle_bringup')
    
    # Hardware camera driver condition
    launch_cam_condition = IfCondition(
        PythonExpression(["'", hardware_type, "' == 'real' and '", enable_ai, "' == 'true'"])
    )
    
    return LaunchDescription([
        
        # Add the arguments
        DeclareLaunchArgument(
            'hardware_type',
            default_value='real',
            description='Type of hardware: "real" or "simulated"'
        ),
        DeclareLaunchArgument(
            'show_sim',
            default_value='true',
            description='Whether to launch Gazebo/RViz when in simulated mode: true or false'
        ),
        DeclareLaunchArgument(
            'enable_ai',
            default_value='false',
            description='Whether to use UFLD (AI) or OpenCV for lane detection'
        ),
        DeclareLaunchArgument(
            'enable_drive',
            default_value='true',
            description='Whether automatic driving will be enabled'
        ),
        
        # Include the Base Hardware and forward the arguments to it
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'hardware.launch.py')
            ),
            launch_arguments=[
                ('hardware_type', hardware_type),
                ('show_sim', show_sim)
            ]
        ),

        # Camera driver
        Node(
            package='vehicle_perception', 
            executable='camera_driver_node',
            name='camera_driver_node',
            condition=launch_cam_condition,
            output='screen',
        ),

        # Lane detection 
        Node(
            package='lane_perception',
            executable='lane_perception_node',
            name='lane_perception_node',
            condition=LaunchConfigurationEquals('enable_ai', 'false'),
            parameters=[{'use_sim': PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
            output='screen'
        ),
        
        # AI Perception
        Node(
            package='vehicle_perception',
            executable='ufld_node',
            name='ufld_node',
            condition=LaunchConfigurationEquals('enable_ai', 'true'),
            output='screen'
        ),
        
        
        # Steering Bridge
        Node(
            package='vehicle_hardware',
            executable='steering_bridge_node',
            name='steering_bridge_node',
            parameters=[{'speed': 0.3, 'kp': 0.005}],
            condition=LaunchConfigurationEquals('enable_drive', 'true'),
            output='screen'
        )
    ])