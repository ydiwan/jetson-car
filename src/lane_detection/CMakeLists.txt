cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)



# Include directories
include_directories(
  include
)

add_executable(lane_detection_node 
  src/lane_detection_node.cpp
  src/lane_detector.cpp
  src/ld_preprocessor.cpp
  src/ld_scanner.cpp
  src/ld_postprocessor.cpp
  src/ld_conf_calculator.cpp)
  
target_include_directories(lane_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(lane_detection_node PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies( lane_detection_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(lane_detection_node ${OpenCV_LIBS} ${EXTRA_LIBS})

install(TARGETS
  lane_detection_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
