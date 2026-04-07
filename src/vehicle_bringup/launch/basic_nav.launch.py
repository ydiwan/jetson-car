import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

def generate_launch_description():
    # Launch Configuration
    hardware_type = LaunchConfiguration('hardware_type', default='real')
    show_sim = LaunchConfiguration('show_sim', default='false')
    enable_nav2 = LaunchConfiguration('enable_nav2', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file', default='cyber_city_with_brim.yaml')
    rviz_config = LaunchConfiguration('rviz_config', default='nav2.rviz')
    use_controller = LaunchConfiguration('use_controller', default='true')
    
    use_sim_time_param = {'use_sim_time': use_sim_time}
    bringup_dir = get_package_share_directory('vehicle_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    # LaunchA Args
    arg_hardware_type = DeclareLaunchArgument('hardware_type', default_value='real')
    arg_show_sim = DeclareLaunchArgument('show_sim', default_value='false')
    arg_enable_nav2 = DeclareLaunchArgument('enable_nav2', default_value='true')
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    arg_map_file = DeclareLaunchArgument('map_file', default_value='cyber_city_with_brim.yaml')
    arg_rviz_config = DeclareLaunchArgument('rviz_config', default_value='nav2.rviz')
    arg_use_controller = DeclareLaunchArgument('use_controller', default_value='true', description='Enable custom USB joystick teleop')


    # Launch hardware and lane detection
    lane_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', '3dlane.launch.py')),
        launch_arguments=[
            ('hardware_type', hardware_type),
            ('use_sim_time', use_sim_time),
            ('show_sim', show_sim),
            ('rviz_config', rviz_config),
            ('use_controller', use_controller)
        ]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'True'
        }.items(),
        condition=LaunchConfigurationEquals('enable_nav2', 'true')
    )

    delayed_nav2_launch = TimerAction(
        period=8.0, 
        actions=[nav2_launch]
    )
    
    # Nodes
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': PathJoinSubstitution([bringup_dir, 'worlds/materials/textures', map_file])},
            use_sim_time_param
        ],
        output='screen'
    )

    lifecycle_manager_map_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'autostart': True}, {'node_names': ['map_server']}, use_sim_time_param]
    )

    return LaunchDescription([
        # Arguments
        arg_hardware_type,
        arg_show_sim,
        arg_enable_nav2,
        arg_use_sim_time,
        arg_map_file,
        arg_rviz_config,

        # Launches
        lane_pipeline_launch,
        delayed_nav2_launch,

        # Nodes
        map_server_node,
        lifecycle_manager_map_node
    ])