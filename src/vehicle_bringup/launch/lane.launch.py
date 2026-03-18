import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments
    hardware_type = DeclareLaunchArgument(
        'hardware_type',
        default_value='real',
        description='Type of hardware: "real" or "simulated"'
    )
    
    show_sim = DeclareLaunchArgument(
        'show_sim',
        default_value='false',
        description='Whether to launch Gazebo/RViz when in simulated mode'
    )

    # Get launch arguments
    hardware_type = LaunchConfiguration('hardware_type')
    show_sim = LaunchConfiguration('show_sim')
    
    # Path to bringup package
    bringup_dir = get_package_share_directory('vehicle_bringup')
    
    return LaunchDescription([
        
        # Add the arguments
        hardware_type,
        show_sim,
        
        # Include the Base Hardware and forward the arguments to it
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'hardware.launch.py')
            ),
            launch_arguments={
                'hardware_type': hardware_type,
                'show_sim': show_sim
            }.items()
        ),

        # Camera driver
        Node(
            package='vehicle_perception', 
            executable='camera_lane_node',
            name='camera_lane_node',
            output='screen'
        ),

        # Lane detection 
        Node(
            package='lane_detection',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen'
        )
    ])