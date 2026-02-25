from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vicon_reader',
            executable='vicon_reader_node',
            name='vicon_reader_node',
            output='screen'
        ),
        Node(
            package='vehicle_controller',
            executable='vehicle_controller_node',
            name='vehicle_controller_node',
            arguments=[
                '--ros-args', 
                '--log-level', 'vehicle_controller_node:=debug',
                '--log-level', 'vehicle_class:=debug'  
            ],
            output='screen'
        ),
        Node(
            package='gpio_node',
            executable='gpio_node',
            name='gpio_node',
            arguments=[
                '--ros-args', 
                '--log-level', 'gpio_node:=info'
            ],
            output='screen'
        ),
        Node(
            package='vehicle_controller',
            executable='waypoint_loader_node',
            name='waypoint_loader',
            parameters=[{
                'csv_file': "/home/jetson-car/jetson_car/src/vehicle_controller/src/coordinates.csv",
                'publish_frequency' : 10.0    
            }],
            output='screen'
        ),
    ])