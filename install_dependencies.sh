#!/bin/bash

# Install ROS2 Humble dependencies
echo "Installing ROS2 Humble dependencies..."

# Update package list
sudo apt update

# Install navigation2 packages
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-amcl \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-recoveries \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-waypoint-follower \
  ros-humble-nav2-rviz-plugins

# Install robot model packages
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros2-control-demos

# Install perception packages
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-image-pipeline \
  libopencv-dev \
  python3-opencv

# Install control packages
sudo apt install -y \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers

# Install tf2 packages
sudo apt install -y \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-eigen

# Install visualization tools
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins \
  ros-humble-rviz-visual-tools

# Install SLAM packages (optional)
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-cartographer \
  ros-humble-cartographer-ros

# Install additional utilities
sudo apt install -y \
  ros-humble-teleop-twist-keyboard \
  ros-humble-teleop-twist-joy \
  ros-humble-joy \
  ros-humble-robot-localization \
  ros-humble-imu-tools

echo "Dependencies installed successfully!"
