# Vicon Reader Package

Gets positioning data from Vicon System and publishes it for other nodes.

## Overview

ViconPacketReader.cpp creates a UDP socket to recieve UDP packets from the Vicon system.
Extracts 6 values:
[1] = x
[2] = y
[3] = z
[4] = roll
[5] = pitch
[6] = yaw

vicon_reader_node.cpp continually calls ViconPacketReader to update positioning data. COnverts raw data to ROS2 PoseStamped messages to be used by other nodes. Publishes to the "vicon_pose" topic.

vicon_subscriber_node.cpp I believe is not needed. Will either delete or update this section.

## Usage

```bash
cd ros2_ws/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

To communicate with vehicle_controller_node, it must be launched in sudo:
```bash
sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source /home/car/ros2_ws/install/setup.bash && ros2 run vicon_reader vicon_reader_node'
```
