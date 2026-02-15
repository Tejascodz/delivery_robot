cmake_minimum_required(VERSION 3.8)
project(delivery_robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Create a library from obstacle_detection.cpp
add_library(obstacle_detection_lib SHARED
  src/obstacle_detection.cpp
)

ament_target_dependencies(obstacle_detection_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)

# Create the executable node
add_executable(obstacle_detection_node
  src/obstacle_detection_node.cpp
)

target_link_libraries(obstacle_detection_node
  obstacle_detection_lib
)

ament_target_dependencies(obstacle_detection_node
  rclcpp
)

# Install targets
install(TARGETS
  obstacle_detection_lib
  obstacle_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export the library
ament_export_include_directories(include)
ament_export_libraries(obstacle_detection_lib)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers so other packages can use them
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
