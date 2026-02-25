# Video Nodes Package

A ROS2 package for streaming video from cameras using GStreamer, providing both publisher and subscriber nodes for real-time video feed processing.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Nodes](#nodes)
  - [video_publisher](#video_publisher)
  - [video_subscriber](#video_subscriber)
- [Usage Examples](#usage-examples)
- [Troubleshooting](#troubleshooting)

## Overview

This package provides a high-performance video streaming solution for ROS2 robots. The `video_publisher` node captures video from V4L2-compatible cameras using GStreamer pipelines and publishes raw images. The `video_subscriber` node can display these images for debugging and monitoring purposes.

## Features

- Low-latency video streaming using GStreamer
- Configurable resolution and framerate
- Support for multiple camera devices
- Real-time display capability for debugging
- Optimized for embedded systems

## Dependencies

### ROS2 Packages

- `rclcpp`
- `sensor_msgs`
- `cv_bridge`
- `std_msgs`

### System Libraries

- OpenCV 4.x
  
- GStreamer 1.0 and plugins:
  
  ```bash
  sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \                     gstreamer1.0-plugins-base gstreamer1.0-plugins-good \                     gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
  ```
  
- V4L2 utilities:
  
  ```bash
  sudo apt-get install v4l-utils
  ```
  

## Installation

1. Build the package:
  
  ```bash
  cd ~/ros2_ws
  colcon build --packages-select video_nodes
  ```
  
2. Source the workspace:
  
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```
  

## Nodes

### video_publisher

Captures video from a camera device and publishes it as ROS2 Image messages.

#### Published Topics

- **`camera/image_raw`** (`sensor_msgs/msg/Image`)
  - Raw BGR8 video feed from the camera
  - Header includes timestamp and frame_id "camera_frame"

#### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `camera_index` | int | 0   | Camera device index (e.g., 0 for `/dev/video0`) |
| `width` | int | 640 | Video capture width in pixels |
| `height` | int | 480 | Video capture height in pixels |
| `fps` | int | 60  | Frames per second |

### video_subscriber

Subscribes to video feed and displays it using OpenCV for debugging purposes.

#### Subscribed Topics

- **`camera/image_raw`** (`sensor_msgs/msg/Image`)
  - Expects BGR8 encoded images
  - Uses `SensorDataQoS` for best-effort, low-latency delivery

## Usage Examples

### Finding Available Cameras

```bash
# List all video devices
v4l2-ctl --list-devices

# Example output:
USB2.0 camera: USB2.0 camera (usb-0000:00:14.0-5):
        /dev/video0
        /dev/video1
```

### Checking Camera Capabilities

```bash
# List supported formats and resolutions
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

### Running the Publisher

```bash
# Default settings (640x480 @ 60fps on /dev/video0)
ros2 run video_nodes video_publisher

# Custom resolution and framerate
ros2 run video_nodes video_publisher --ros-args \
  -p camera_index:=0 \
  -p width:=1280 \
  -p height:=720 \
  -p fps:=30
```

### Running the Subscriber

```bash
# Display the video feed
ros2 run video_nodes video_subscriber
```

### Checking Video Stream

```bash
# View topic info
ros2 topic info /camera/image_raw

# Check publishing rate
ros2 topic hz /camera/image_raw
```

## Troubleshooting

### Camera not found

- Check camera connection: `ls /dev/video*`
- Add user to video group: `sudo usermod -a -G video $USER` (logout required)
- Verify camera with: `v4l2-ctl --device=/dev/video0 --info`

### No image in subscriber

- Check publisher is running: `ros2 node list`
- Verify topic is publishing: `ros2 topic hz /camera/image_raw`
- For SSH sessions, ensure DISPLAY enviroment variable is set

### Camera is not working after restarting the video_publisher node
- This happen sometimes after killing (ctrl^c) the node, for some reason the UVC driver doesn't close the Camera device.
-  Run this in the terminal to restart the UVC driver:
```
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo