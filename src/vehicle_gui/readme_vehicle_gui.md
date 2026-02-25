# Vehicle GUI Package

For giving waypoints to the vehicle through a graphical user interface (GUI)

## Overview

maps folder holds the CyberCity map, currently not able to be found in the maps folder, so be sure the cyber_city_map.png is also in the root folder

grid_manager.py loads the image of the city and detects the road so that waypoints can only be placed on the road

path_planner.py is a path finder algorithm that places waypoints between user set waypoints to ensure it follows the roads and does not go off road. Currently not working as intended, so it is not implemented. Current problems include: causes vehicle to only follow path between last two user set waypoints, sharp turns, does not stay in lane

waypoint_gui.py creates the GUI and scales the map to translate to Vicon points. 

## Usage

```bash
cd ros2_ws/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

To communicate with vehicle_controller_node, it must be launched in sudo:
```bash
sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source /home/car/ros2_ws/install/setup.bash && DISPLAY=$DISPLAY ros2 run vehicle_gui waypoint_gui'
```

User selects the desired waypoints in order they want the vehicle to traverse and click 'Send Waypoints' button to send to publish. Use 'Clear Waypoints' button to clear selected waypoints.