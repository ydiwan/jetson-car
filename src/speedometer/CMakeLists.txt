cmake_minimum_required(VERSION 3.8)
project(speedometer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build
add_executable(speedometer_node src/speedometer_node.cpp)

# Include directories
target_include_directories(speedometer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Dependencies
ament_target_dependencies(speedometer_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install
install(TARGETS
  speedometer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have any
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()