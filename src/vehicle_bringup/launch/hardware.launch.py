import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Parse URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('vehicle_bringup'), 'urdf', 'jetson_car.urdf.xacro']
            ),
            ' ',
            'use_sim:=false' # Force physical hardware mode
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Get the path to controllers.yaml
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('vehicle_hardware'), 'config', 'controllers.yaml']
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen',
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Ackermann Steering Controller
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
    )

    # Wait for broadcaster before starting controller
    delay_ackermann_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_ackermann_spawner,
    ])