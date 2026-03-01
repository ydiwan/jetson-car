cmake_minimum_required(VERSION 3.8)
project(video_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)  # Added OpenCV find_package

# Print OpenCV info for debugging
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

add_executable(video_publisher src/video_publisher.cpp)
target_include_directories(video_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  
target_compile_features(video_publisher PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  video_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV  # Added OpenCV dependency
)
# Critical addition: explicitly link against OpenCV
target_link_libraries(video_publisher ${OpenCV_LIBS})

add_executable(video_subscriber src/video_subscriber.cpp)
ament_target_dependencies(video_subscriber
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
# Also link the subscriber with OpenCV
target_link_libraries(video_subscriber ${OpenCV_LIBS})

install(TARGETS
  video_publisher
  video_subscriber
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