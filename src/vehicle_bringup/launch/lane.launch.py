import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Configuration
    hardware_type = LaunchConfiguration('hardware_type', default='real')
    show_sim = LaunchConfiguration('show_sim', default='false')
    enable_ai = LaunchConfiguration('enable_ai',  default='false')
    enable_drive = LaunchConfiguration('enable_drive', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default='hardware.rviz')

    use_sim_time_param = {'use_sim_time': use_sim_time}
    bringup_dir = get_package_share_directory('vehicle_bringup')
    
    launch_cam_condition = IfCondition(PythonExpression(["'", hardware_type, "' == 'real' and '", enable_ai, "' == 'true'"]))

    # Launch Args
    arg_hardware_type = DeclareLaunchArgument('hardware_type', default_value='real')
    arg_show_sim = DeclareLaunchArgument('show_sim', default_value='true')
    arg_enable_ai = DeclareLaunchArgument('enable_ai', default_value='false')
    arg_enable_drive = DeclareLaunchArgument('enable_drive', default_value='true')
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    arg_rviz_config = DeclareLaunchArgument('rviz_config', default_value='hardware.rviz')


    # Launch Hardware
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'hardware.launch.py')),
        launch_arguments=[
            ('hardware_type', hardware_type),
            ('show_sim', show_sim),
            ('use_sim_time', use_sim_time),
            ('rviz_config', rviz_config)
        ]
    )

    # Nodes
    camera_driver_node = Node(
        package='vehicle_perception', 
        executable='camera_driver_node',
        name='camera_driver_node',
        condition=launch_cam_condition,
        output='screen',
    )

    lane_perception_node = Node(
        package='lane_perception',
        executable='lane_perception_node',
        name='lane_perception_node',
        condition=LaunchConfigurationEquals('enable_ai', 'false'),
        parameters=[{'use_sim': PythonExpression(["'", hardware_type, "' == 'simulated'"])}, use_sim_time_param],
        output='screen'
    )
    
    ufld_node = Node(
        package='vehicle_perception',
        executable='ufld_node',
        name='ufld_node',
        condition=LaunchConfigurationEquals('enable_ai', 'true'),
        parameters=[use_sim_time_param],
        output='screen'
    )
    
    steering_bridge_node = Node(
        package='vehicle_hardware',
        executable='steering_bridge_node',
        name='steering_bridge_node',
        parameters=[{'speed': 0.3, 'kp': 0.05}, use_sim_time_param],
        remappings=[
            ('/cmd_vel', '/ackermann_steering_controller/reference_unstamped'),
            ('cmd_vel', '/ackermann_steering_controller/reference_unstamped')
        ],
        condition=LaunchConfigurationEquals('enable_drive', 'true'),
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        arg_hardware_type,
        arg_show_sim,
        arg_enable_ai,
        arg_enable_drive,
        arg_use_sim_time,
        arg_rviz_config,
        
        # Launches
        hardware_launch,
        
        # Nodes
        camera_driver_node,
        lane_perception_node,
        ufld_node,
        steering_bridge_node
    ])