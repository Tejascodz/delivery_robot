cmake_minimum_required(VERSION 3.8)
project(delivery_robot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(delivery_robot_perception REQUIRED)

# Include directories
include_directories(include)

# Create delivery_manager executable
add_executable(delivery_manager
  src/delivery_manager.cpp
)

# Specify dependencies for the executable
ament_target_dependencies(delivery_manager
  rclcpp
  rclcpp_action
  nav2_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  delivery_robot_perception
)

# Install executables
install(TARGETS
  delivery_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

# Export package dependencies in the CMake environment
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  delivery_robot_perception
)

# Export include directories
ament_export_include_directories(include)

# Add linting tests (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate CMake config files
ament_package()
