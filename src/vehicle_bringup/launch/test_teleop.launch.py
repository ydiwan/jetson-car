import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1. Get URDF path
    urdf_file = os.path.join(
        get_package_share_directory('vehicle_bringup'),
        'urdf',
        'jetson-car.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot State Publisher 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Odometry Estimator 
        Node(
            package='sensor_fusion',
            executable='odometry_estimator_node',
            name='odometry_estimator_node',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        
        # Publishes dummy zero-states for the continuous wheel joints
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Hardware Interfaces
        Node(
            package='hardware_interface', 
            executable='ackermann_kinematics_node',
            name='ackermann_kinematics_node',
            output='screen'
        ),
        Node(
            package='hardware_interface',
            executable='maestro_interface_node',
            name='maestro_interface_node',
            output='screen'
        ),
    ])