cmake_minimum_required(VERSION 3.16)
project(vicon_display)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)                    
find_package(tf2_geometry_msgs REQUIRED) 

# Include directories
include_directories(include)

# Add subscriber executable
add_executable(vicon_display_node src/vicon_display_node.cpp)
ament_target_dependencies(vicon_display_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install executables
install(TARGETS 
  vicon_display_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()