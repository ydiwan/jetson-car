# ROS Node Graph Overview

## Layer 1: Hardware Drivers (The physical interface)
* camera_lane_node

    Publishes: /camera/lane/image_raw (sensor_msgs/Image)

* camera_driver_node

    Publishes: /camera/driver/image_raw (sensor_msgs/Image)

* maestro_interface_node

    Subscribes: /vehicle/steering_angle (std_msgs/Float32)

    Action: Translates ROS inputs to Pololu serial commands for the front steering servo.

* gpio_node

    Subscribes: gpio/pwm_left (std_msgs/Int32), gpio/pwm_right (std_msgs/Int32)

    Action: Generates 3.3V Active-Low PWM signals directly from the Jetson's GPIO pins to drive the rear motors.

* vicon_bridge (Runs on separate Pi)

    Publishes: /vicon_pose (geometry_msgs/PoseStamped at 100Hz)

## Layer 2: Perception (The AI models)
* lane_detector_node

    Subscribes: /camera/lane/image_raw

    Publishes: /perception/lane_path (nav_msgs/Path), /perception/lane_delta (std_msgs/Int32)

* obstacle_detector_node

    Subscribes: /camera/lane/image_raw

    Publishes: /perception/forward_clearance (std_msgs/Float32)

* traffic_monitor_node

    Subscribes: /camera/driver/image_raw

    Publishes: /perception/traffic_state (Custom msg: contains light color, stop sign presence, speed limit value)

## Layer 3: Localization (Sensor Fusion)
* odometry_calculator_node

    Subscribes: /vehicle/wheel_ticks

    Publishes: /vehicle/odom (nav_msgs/Odometry)

* ekf_fusion_node (Using robot_localization package)

    Subscribes: /vehicle/odom, /vicon/jetson_car/pose

    Publishes: /odometry/filtered (nav_msgs/Odometry), TF tree updates (map -> base_link)

## Layer 4: Planning & Control (The Brain)
* behavior_planner_node

    Subscribes: /perception/traffic_state, /perception/forward_clearance, /cyber_city/traffic_lights (V2I), /v2v/telemetry (V2V)

    Publishes: /planner/target_state (Custom msg: enum for CRUISING, YIELDING, etc.), /planner/target_speed (std_msgs/Float32)

* local_trajectory_node

    Subscribes: /perception/lane_path, /planner/target_speed, /odometry/filtered

    Publishes: /cmd_vel (geometry_msgs/Twist)

* ackermann_kinematics_node

    Subscribes: /cmd_vel

    Publishes: /vehicle/wheel_speeds, /vehicle/steering_angle

    Action: Applies the electronic differential math for the dual rear motors and front steering angle.

## Layer 5: V2X & Telemetry
* v2x_broadcaster_node

    Subscribes: /odometry/filtered, /cmd_vel, /planner/target_state

    Publishes: /v2v/jetson_car_1/telemetry (Custom msg) to the global domain.

* dashboard_streamer_node

    Subscribes: Key image topics, pose, and state data.

    Publishes: Compressed images and consolidated states to a WebSocket via rosbridge_server for the UI/UX dashboard.

