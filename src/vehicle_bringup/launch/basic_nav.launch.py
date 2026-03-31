import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Declare launch arguments
    hardware_type = LaunchConfiguration('hardware_type', default='real')
    show_sim = LaunchConfiguration('show_sim', default='false')
    enable_nav2 = LaunchConfiguration('enable_nav2', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Package Directories
    bringup_dir = get_package_share_directory('vehicle_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Path to the Nav2 Configuration
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("vehicle_bringup"), "rviz", "nav2.rviz"]
    )
    
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
            description='Whether to launch Gazebo/RViz when in simulated mode'
        ),
        DeclareLaunchArgument(
            'enable_nav2',
            default_value='true',
            description='Whether automatic driving will be enabled via Nav2'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use gazebo clock'    
        ),
        
        # Base Hardware
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(bringup_dir, 'launch', 'hardware.launch.py')
        #     ),
        #     launch_arguments=[
        #         ('hardware_type', hardware_type),
        #         ('show_sim', show_sim),
        #         ('use_sim_time', use_sim_time)
        #     ]
        # ),

        # 3D Lane Perception Pipeline
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', '3dlane.launch.py')
            ),
            launch_arguments=[
                ('hardware_type', hardware_type),
                ('use_sim_time', use_sim_time),
                ('show_sim', show_sim)
            ]
        ),

        # Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'autostart': 'True'
            }.items(),
            condition=LaunchConfigurationEquals('enable_nav2', 'true')
        ),
        
        # Broadcast the map image to RViz
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': os.path.join(bringup_dir, 'worlds/materials/textures', 'cyber_city_map.yaml')}],
            output='screen'
        ),

        # Lifecycle manager specifically to activate the map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'autostart': True}, {'node_names': ['map_server']}, use_sim_time_param]
        ),

        # Static TF to lock the map frame to the odom frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=[
                '--x', '2.4', 
                '--y', '-1.1', 
                '--z', '0.0',
                '--yaw', '-1.57', 
                '--pitch', '0.0', 
                '--roll', '0.0',
                '--frame-id', 'map', 
                '--child-frame-id', 'odom'
            ]
        )
    ])