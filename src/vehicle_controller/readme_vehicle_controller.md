# Vehicle Controller Package

The central package for the vehicle.

## Overview

vehicle.cpp defines functions to control the vehicle like speed and steering

vehicle_controller_node.cpp creates a node for controlling the vehicle.

waypoint_loader.cpp and waypoint_loader_node are for loading defined waypoints from a .csv file, coordinates.csv (primarily for testing)

## Usage
Need four seperate terminals:

In each terminal run:
```bash
cd ros2_ws/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

In terminal 1: (launches gpio_bridge, needs to be run with sudo)
```bash
cd src/vehicle_controller/src
sudo ./gpio_bridge
```

In terminal 2: (launches vicon_reader_node)
```bash
ros2 run vicon_reader vicon_reader_node'
```

In terminal 3: (launches vehicle_controller_node)
```bash
ros2 run vehicle_controller vehicle_controller_node'
```

In terminal 4: (launches waypoint_gui_node, after a few seconds the CyberCity map should open and allow user to select waypoints to send)
```bash
ros2 run vehicle_gui waypoint_gui'
```



