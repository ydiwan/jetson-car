#!/bin/bash

echo "========================================"
echo " Starting Jetson Car Hardware Setup"
echo "========================================"

echo "[1/3] Installing Required Python USB Libraries..."
sudo apt update
sudo apt install -y python3-pip python3-usb evtest
# Break-system-packages is required on Ubuntu 22.04+ to install globally for all users
sudo pip3 install pyusb --break-system-packages

echo "[2/3] Configuring Permanent UDEV Rules..."
# 1. Logitech F710 Controller (Vendor 046d) - Unlocks raw USB read for joy_node
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", MODE="0666"' | sudo tee /etc/udev/rules.d/99-logitech-gamepad.rules

# 2. Maestro Servo Controller (ACM Serial) - Unlocks the steering serial port without needing dialout groups
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1ffb", MODE="0666"' | sudo tee /etc/udev/rules.d/99-maestro.rules

# Force Linux to apply the new hardware rules immediately
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[3/3] Setting ROS 2 Environment Variables..."
# Adds the Domain ID to bashrc if it isn't there already
if ! grep -q "ROS_DOMAIN_ID=25" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=25" >> ~/.bashrc
    echo "Added ROS_DOMAIN_ID=25 to ~/.bashrc"
else
    echo "ROS_DOMAIN_ID is already set in ~/.bashrc"
fi

echo "========================================"
echo " Setup Complete! "
echo " Please run: source ~/.bashrc"
echo " You can now launch the car without sudo."
echo "========================================"