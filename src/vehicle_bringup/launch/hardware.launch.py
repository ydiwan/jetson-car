import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,  IncludeLaunchDescription,  TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Launch configuration
    hardware_type = LaunchConfiguration('hardware_type', default='real')
    show_sim = LaunchConfiguration('show_sim', default='false')
    spawn_x = LaunchConfiguration('x', default='2.4')
    spawn_y = LaunchConfiguration('y', default='-1.1')
    spawn_z = LaunchConfiguration('z', default='0.15')
    spawn_yaw = LaunchConfiguration('yaw', default='-1.57')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    use_sim_time_param = {'use_sim_time': use_sim_time}

    # Paths and conditions
    pkg_bringup = get_package_share_directory('vehicle_bringup')
    pkg_hardware = get_package_share_directory('vehicle_hardware')
    pkg_sensor_fusion = get_package_share_directory('sensor_fusion')

    world_file = os.path.join(pkg_bringup, 'worlds', 'cyber_city_custom.sdf')
    ekf_config_path = os.path.join(pkg_sensor_fusion, 'config', 'ekf.yaml')
    rviz_config_file = os.path.join(pkg_bringup, 'rviz', 'hardware.rviz')
    controllers_file = os.path.join(pkg_hardware, 'config', 'controllers.yaml')
    
    try:
        gz_launch_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    except PackageNotFoundError:
        gz_launch_path = "gazebo_not_installed"

    # Conditions
    is_real = LaunchConfigurationEquals('hardware_type', 'real')
    is_sim = LaunchConfigurationEquals('hardware_type', 'sim')
    
    launch_sim_gui_condition = IfCondition(
        PythonExpression(["'", hardware_type, "' == 'sim' and '", show_sim, "' == 'true'"])
    )

    # Sim param for xacro
    is_sim_str = PythonExpression(["'true' if '", hardware_type, "' == 'sim' else 'false'"])

    # Compile the URDF
    robot_desc = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('vehicle_bringup'), 'urdf', 'jetson_car.urdf.xacro']), ' ',
        'use_sim:=', is_sim_str
    ])

    # Declaire Arguments
    arg_hardware_type = DeclareLaunchArgument('hardware_type', default_value='real', description='Target hardware: "real" or "sim"')
    arg_show_sim = DeclareLaunchArgument('show_sim', default_value='true', description='Launch Gazebo/RViz GUI (true/false)')
    arg_x = DeclareLaunchArgument('x', default_value='2.4', description='X spawn coordinate')
    arg_y = DeclareLaunchArgument('y', default_value='-1.1', description='Y spawn coordinate')
    arg_z = DeclareLaunchArgument('z', default_value='0.15', description='Z spawn coordinate')
    arg_yaw = DeclareLaunchArgument('yaw', default_value='-1.57', description='Yaw spawn rotation')
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use Gazebo clock')
    
    # Always on nodes
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}, use_sim_time_param],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=launch_sim_gui_condition,
        parameters=[use_sim_time_param],
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Real hardware nodes
    real_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controllers_file],
        output='screen',
        condition=is_real
    )
    
    gpio_node = Node(
        package='sensor_fusion',
        executable='gpio_node',
        name='gpio_node',
        condition=is_real,
        output='screen'
    ),

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        condition=is_real,
        output='screen',
        parameters=[ekf_config_path, use_sim_time_param]
    )

    # Simulation nodes
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        condition=launch_sim_gui_condition
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'jetson_car',
            '-allow_renaming', 'false',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw
        ],
        condition=is_sim,
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/lane/raw_video@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/jetson_car/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        condition=is_sim,
        output='screen'
    )

    sim_ground_truth = Node(
        package='sensor_fusion',
        executable='sim_ground_truth_node',
        name='sim_ground_truth_node',
        parameters=[use_sim_time_param],
        condition=is_sim,
        output='screen'
    )

    # Controller Spawners
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        arg_hardware_type,
        arg_show_sim,
        arg_x,
        arg_y,
        arg_z,
        arg_yaw,
        arg_use_sim_time,
        
        # Core
        rsp_node,
        rviz_node,
        
        # Real Hardware
        real_controller_manager,
        ekf_node,
        
        # Simulation
        gazebo_sim,
        spawn_entity,
        clock_bridge,
        sim_ground_truth,
        
        # Shared Spawners
        delayed_spawners
    ])