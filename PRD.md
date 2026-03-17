# Product Requirements Document (PRD): Autonomous Jetson-Car

## 1. Project Overview
**Objective:** Develop a 1/10th scale autonomous vehicle capable of navigating an open Cyber City testbed. The vehicle will utilize Ackermann steering and rely on a decoupled, vision-based perception stack combined with external Vicon motion capture data for precise localization, intelligent decision-making, and multi-agent V2X communication.

## 2. Hardware Specifications
* **Compute Engine:** NVIDIA Jetson Orin Nano (Primary ROS2 node execution, AI inference, Rear Motor GPIO PWM Control).
* **Vision Sensors:** 2x See3CAM CU30 USB Cameras.
    * *Camera 1 (Low Mount):* Dedicated to lane detection and immediate frontal obstacle detection.
    * *Camera 2 (Driver Perspective):* Dedicated to traffic light state detection, stop signs, and speed limit sign reading.
* **Actuation:**
    * *Drive:* 2x DFRobot FIT0441 Brushless DC Motors (12V, 159RPM) with encoders (rear-wheel drive).
        * Hardware Info 1 (Active-Low): Motors require a 0% duty cycle to run at max speed, and a 100% duty cycle to stop.

        * Hardware Info 2 (Direction Limits): Right motor DIR pin is disconnected/hardwired; the chassis currently operates in a Forward-Only configuration.
    * *Steering:* Front axle steering servo (Ackermann configuration).
* **Low-Level Control:** Pololu Micro Maestro Controller (Dedicated exclusively to the front steering servo).
* **Localization Ingestion:** Raspberry Pi receiving Vicon system UDP/TCP stream.

## 3. Software Stack & Ecosystem
* **OS:** Ubuntu 22.04
* **Middleware:** ROS2 Humble Hawksbill
* **Computer Vision/AI:** OpenCV, TensorRT.
* **External Interfaces:** Vicon DataStream SDK.

## 4. Core Functional Requirements

### 4.1. Localization & State Estimation
* **FR-1:** Ingest `geometry_msgs/msg/PoseStamped` data at 100Hz from the Vicon bridge node.
* **FR-2:** Calculate wheel odometry.
* **FR-3:** Fuse Vicon pose data with wheel odometry via an Extended Kalman Filter (EKF).

### 4.2. Perception
* **FR-4 (Lane Tracking):** Process the low-mount camera feed to identify lane boundaries and generate a target centerline path (`nav_msgs/Path`).
* **FR-5 (Traffic Sign/Light):** Process the perspective camera feed to identify traffic lights, stop signs, and speed limit values.
* **FR-6 (Obstacle Detection):** Detect dynamic and static obstacles in the immediate forward path.

### 4.3. Planning & Control
* **FR-7 (Behavioral State Machine):** Transition between states (e.g., `LANE_FOLLOWING`, `STOPPED_AT_LIGHT`) based on perception inputs.
* **FR-8 (Local Trajectory):** Generate `geometry_msgs/Twist` (`cmd_vel`) commands to track the centerline.
* **FR-9 (Ackermann Kinematics):** Translate `cmd_vel` into specific steering servo angles and individual rear-wheel speeds using an Electronic Differential algorithm. Rear-wheel speeds must be inverted to a 0-1000 scale to satisfy Active-Low motor hardware

### 4.4. V2X Communication (V2V & V2I)
* **FR-10 (V2I Traffic State):** Subscribe to a central Cyber City infrastructure topic over the ROS2 Wi-Fi domain (`ROS_DOMAIN_ID=25`).
* **FR-11 (V2V Telemetry Broadcast):** Broadcast current pose, velocity, and active behavioral state to a shared namespace.
* **FR-12 (V2V Collision Avoidance):** Ingest V2V telemetry from other vehicles to adjust target speeds at shared intersections.

### 4.5. Telemetry & Dashboard
* **FR-13 (Live Dashboard):** Support a real-time web dashboard for remote monitoring.
* **FR-14 (Data Streaming):** Publish compressed image streams (`sensor_msgs/CompressedImage`).
* **FR-15 (State Visualization):** Visually plot the vehicle's pose on a 2D map of the testbed alongside active metrics.

## 5. Non-Functional Requirements
* **NFR-1 (Real-Time Performance):** Vision inference pipelines must operate at a minimum of 20 FPS.
* **NFR-2 (Modularity):** All ROS2 nodes must be completely decoupled.
* **NFR-3 (Safety):** Include a manual teleoperation override (gamepad dead-man's switch).

## 6. Simulation Environment (Digital Twin)
* **FR-16 (Gazebo World):** Accurately mirror the physical dimensions and markings of the testbed.
* **FR-17 (Vehicle URDF/SDF):** Include an Ackermann steering plugin matching the physical hardware.
* **FR-18 (Sensor Plugins):** Include virtual RGB camera and ground-truth pose publisher plugins.