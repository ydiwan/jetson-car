 
cmake_minimum_required(VERSION 3.8)
project(vehicle_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# Define compilation flags
add_compile_options(-Wall -Wextra -Wpedantic)
  add_definitions(-DNO_PIGPIOD)
  set(EXTRA_INCLUDES "")
  set(EXTRA_LIBS "")


# Include directories
include_directories(
  include
  ${EXTRA_INCLUDES}
)

# Declare the executables
add_executable(vehicle_controller_node
  src/vehicle_controller_node.cpp
  src/vehicle.cpp
  src/msc_if.cpp
)

add_executable(waypoint_loader_node
  src/waypoint_loader_node.cpp
  src/waypoint_loader.cpp
)

# Add dependencies
ament_target_dependencies(vehicle_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

ament_target_dependencies(waypoint_loader_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Link the executables to the necessary libraries
target_link_libraries(vehicle_controller_node
  ${EXTRA_LIBS}
)

# Install the executables
install(TARGETS
  vehicle_controller_node
  waypoint_loader_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the config directory with the waypoints.csv file
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY
  include/
  DESTINATION include/
)

# Set up package
ament_package()
