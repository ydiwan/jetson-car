#!/bin/bash

echo "========================================="
echo " Setting up Autonomous Ground Vehicle Demo"
echo "========================================="

# Navigate to the workspace
WORKSPACE_DIR="$HOME/jetson-car"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Error: Could not find workspace at $WORKSPACE_DIR"
    exit 1
fi

cd "$WORKSPACE_DIR" || exit

# Build the demo package
echo "--> Building the demo_vicon_navigation package..."
colcon build --packages-select demo_vicon_navigation vehicle_hardware sensor_fusion vehicle_bringup lane_perception

# Add aliases to .bashrc
BASHRC_FILE="$HOME/.bashrc"
echo "--> Configuring aliases in $BASHRC_FILE..."

# Brain Alias (For the Lab PC)
if ! grep -q "alias run_demo_brain=" "$BASHRC_FILE"; then
    echo "alias run_demo_brain='source $WORKSPACE_DIR/install/setup.bash && ros2 launch demo_vicon_navigation demo_brain.launch.py'" >> "$BASHRC_FILE"
    echo "  [+] Added 'run_demo_brain' alias."
else
    echo "  [-] 'run_demo_brain' alias already exists."
fi

# Brawn Alias (For the Jetson)
if ! grep -q "alias run_demo_brawn=" "$BASHRC_FILE"; then
    echo "alias run_demo_brawn='source $WORKSPACE_DIR/install/setup.bash && ros2 launch demo_vicon_navigation demo_brawn.launch.py'" >> "$BASHRC_FILE"
    echo "  [+] Added 'run_demo_brawn' alias."
else
    echo "  [-] 'run_demo_brawn' alias already exists."
fi

echo "========================================="
echo " Setup Complete!"
echo " "
echo " IMPORTANT: Run this command to apply changes to your current terminal:"
echo " source ~/.bashrc"
echo " "
echo " To run the Brain (Lab PC):   run_demo_brain"
echo " To run the Brawn (Jetson):   run_demo_brawn"
echo "========================================="