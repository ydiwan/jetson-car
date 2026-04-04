cmake_minimum_required(VERSION 3.8)
project(vehicle_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_library(GPIOD_LIBRARIES gpiod)

# Compile hardware plugin into shared library
add_library(${PROJECT_NAME} SHARED
  src/JetsonCarSystemHardware.cpp
)

# Set the include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against ROS 2 targets
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  {PROJECT_NAME} ${GPIOD_LIBRARIES}
)

# Export the plugin XML so the Controller Manager can find it
pluginlib_export_plugin_description_file(hardware_interface jetson_car_system_hardware.xml)

# Install rules
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()