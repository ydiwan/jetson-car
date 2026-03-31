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
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Path to bringup package
    bringup_dir = get_package_share_directory('vehicle_bringup')
    
    use_sim_time_param = {'use_sim_time': use_sim_time}
    
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
            'enable_drive',
            default_value='false',
            description='Whether automatic driving will be enabled'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use gazebo clock'    
        ),
        
        # Include the Base Hardware and forward the arguments to it
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'hardware.launch.py')
            ),
            launch_arguments=[
                ('hardware_type', hardware_type),
                ('show_sim', show_sim),
                ('use_sim_time', use_sim_time)
            ]
        ),

        # Camera driver
        Node(
            package='vehicle_perception', 
            executable='camera_driver_node',
            name='camera_driver_node',
            condition=LaunchConfigurationEquals('hardware_type', 'real'),
            parameters=[use_sim_time_param],
            output='screen',
        ),
        
        # Lane detection to PointCloud
        Node(
            package='vehicle_perception',
            executable='spatial_lane_node',
            name='spatial_lane_node',
            parameters=[use_sim_time_param],
            output='screen'
        ),
        
        # Steering Bridge
        Node(
            package='vehicle_hardware',
            executable='steering_bridge_node',
            name='steering_bridge_node',
            parameters=[{'speed': 0.3, 'kp': 0.05}, use_sim_time_param],
            condition=LaunchConfigurationEquals('enable_drive', 'true'),
            output='screen'
        )
    ])