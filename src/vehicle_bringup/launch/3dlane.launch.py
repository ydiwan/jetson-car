import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Configuration
    hardware_type = LaunchConfiguration('hardware_type', default='real')
    show_sim = LaunchConfiguration('show_sim', default='false')
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default='3dlane.rviz')
    use_controller = LaunchConfiguration('use_controller', default='true')

    use_sim_time_param = {'use_sim_time': use_sim_time}
    bringup_dir = get_package_share_directory('vehicle_bringup')
    
    # Launch Args
    arg_hardware_type = DeclareLaunchArgument('hardware_type', default_value='real', description='Type of hardware: "real" or "sim"')
    arg_show_sim = DeclareLaunchArgument('show_sim', default_value='true', description='Launch Gazebo/RViz GUI when in simulated mode')
    arg_enable_drive = DeclareLaunchArgument('enable_drive', default_value='false', description='Whether automatic driving will be enabled')
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use gazebo clock')
    arg_rviz_config = DeclareLaunchArgument('rviz_config', default_value='3dlane.rviz', description='RViz config file name')
    arg_use_controller = DeclareLaunchArgument('use_controller', default_value='true', description='Enable custom USB joystick teleop')
    
    # Launch hardware
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'hardware.launch.py')),
        launch_arguments=[
            ('hardware_type', hardware_type),
            ('show_sim', show_sim),
            ('use_sim_time', use_sim_time),
            ('rviz_config', rviz_config),
            ('use_controller', use_controller)
        ]
    )

    # Nodes
    camera_driver_node = Node(
        package='vehicle_perception', 
        executable='camera_driver_node',
        name='camera_driver_node',
        condition=LaunchConfigurationEquals('hardware_type', 'real'),
        parameters=[use_sim_time_param],
        output='screen',
    )
    
    spatial_lane_node = Node(
        package='vehicle_perception',
        executable='spatial_lane_node',
        name='spatial_lane_node',
        parameters=[use_sim_time_param],
        output='screen'
    )
    
    steering_bridge_node = Node(
        package='vehicle_hardware',
        executable='steering_bridge_node',
        name='steering_bridge_node',
        parameters=[{'speed': 0.3, 'kp': 0.05}, use_sim_time_param],
        condition=LaunchConfigurationEquals('enable_drive', 'true'),
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        arg_hardware_type,
        arg_show_sim,
        arg_enable_drive,
        arg_use_sim_time,
        arg_rviz_config,
        arg_use_controller,
        
        # Launches
        hardware_launch,
        
        # Nodes
        camera_driver_node,
        spatial_lane_node,
        steering_bridge_node
    ])