import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Declare launch configurations
    hardware_type = LaunchConfiguration('hardware_type')
    show_sim = LaunchConfiguration('show_sim')
    
    # Get URDF path
    urdf_file = os.path.join(
        get_package_share_directory('vehicle_bringup'),
        'urdf',
        'jetson-car.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Checks to iss if gazebo is available
    try:
        gz_launch_path = os.path.join(
            get_package_share_directory('ros_gz_sim'), 
            'launch', 
            'gz_sim.launch.py'
        )
    except PackageNotFoundError:
        gz_launch_path = "gazebo_not_installed"    
    
    # Only launch gazebo if hardware is simulated and show_sim is true
    launch_gazebo_condition = IfCondition(
        PythonExpression(["'", hardware_type, "' == 'simulated' and '", show_sim, "' == 'true'"])
    )

    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument(
            'hardware_type',
            default_value='simulated',
            description='Type of hardware: "real" or "simulated"'
        ),
        DeclareLaunchArgument(
            'show_sim',
            default_value='true',
            description='Whether to launch Gazebo/RViz when in simulated mode'
        ),
        
        # Robot State Publisher 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Rviz (only if hardware is simulated)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=LaunchConfigurationEquals('hardware_type', 'simulated'),
            output='screen'
        ),
        
        # Odometry Estimator 
        Node(
            package='sensor_fusion',
            executable='odometry_estimator_node',
            name='odometry_estimator_node',
            output='screen'
        ),
        
        # Launch empty gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={'gz_args': '-r empty.sdf'}.items(),
            condition=launch_gazebo_condition
        ),
        
        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', robot_desc, '-name', 'jetson_car', '-allow_renaming', 'true'],
            condition=launch_gazebo_condition,
            output='screen'
        ),
        
        # ROS to gazebo bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            condition=LaunchConfigurationEquals('hardware_type', 'simulated'),
            output='screen'
        ),
        
        # Publishes dummy zero-states for the continuous wheel joints (for rviz)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=LaunchConfigurationEquals('hardware_type', 'simulated'),
            output='screen'
        ),

        # Hardware Interfaces
        Node(
            package='vehicle_hardware', 
            executable='ackermann_kinematics_node',
            name='ackermann_kinematics_node',
            condition=LaunchConfigurationEquals('hardware_type', 'real'),
            output='screen'
        ),
        Node(
            package='vehicle_hardware',
            executable='maestro_interface_node',
            name='maestro_interface_node',
            condition=LaunchConfigurationEquals('hardware_type', 'real'),
            output='screen'
        ),
        
        # GPIO Node
        Node(
            package='vehicle_hardware',
            executable='gpio_node',
            name='gpio_node',
            condition=LaunchConfigurationEquals('hardware_type', 'real'),
            output='screen'
        ),
    ])