# 🤖 Autonomous Delivery Robot - ROS2 Humble

<div align="center">

[![ROS2](https://img.shields.io/badge/ROS2-Humble-34aec5?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue?style=for-the-badge)](LICENSE)
[![Issues](https://img.shields.io/github/issues/Tejascodz/delivery_robot?style=for-the-badge)](https://github.com/Tejascodz/delivery_robot/issues)
[![Stars](https://img.shields.io/github/stars/Tejascodz/delivery_robot?style=for-the-badge)](https://github.com/Tejascodz/delivery_robot/stargazers)
[![Forks](https://img.shields.io/github/forks/Tejascodz/delivery_robot?style=for-the-badge)](https://github.com/Tejascodz/delivery_robot/network/members)

**A Production-Ready Autonomous Delivery Robot System with Intentional Bugs for Learning!**

[View Demo](#-running-the-fixed-system) · [Report Bug](https://github.com/Tejascodz/delivery_robot/issues) · [Request Feature](https://github.com/Tejascodz/delivery_robot/issues)

</div>

---

## 📋 Table of Contents
- [🎯 Your Mission](#-your-mission-fix-the-bugs)
- [📋 System Architecture](#-system-architecture)
- [🛠️ Features](#️-features-when-fixed)
- [📦 Prerequisites](#-prerequisites)
- [🚦 Getting Started](#-getting-started)
- [🐛 Known Bugs](#-known-bugs-to-fix)
- [🔧 How to Contribute](#-how-to-contribute)
- [🏆 Challenge Levels](#-challenge-levels)
- [📊 System Requirements](#-system-requirements)
- [🚀 Running the Fixed System](#-running-the-fixed-system)
- [📁 Project Structure](#-project-structure)
- [📚 Resources](#-resources)
- [🤝 Contributing](#-contributing)
- [📄 License](#-license)

---

## 🎯 Your Mission: Fix the Bugs!

This repository contains a **nearly complete** autonomous delivery robot system built with ROS2 Humble, but there are **intentional bugs and errors** scattered throughout the code. This project is designed as a hands-on learning exercise for ROS2 developers to practice debugging, fixing common issues, and understanding ROS2 architecture.

### Current Issues to Fix:

| Issue | File | Difficulty |
|-------|------|------------|
| **Header Guard Error** | `delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp` | 🌱 Beginner |
| **Missing Includes** | `delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp` | 🌱 Beginner |
| **Incomplete Function Declarations** | `delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp` | 🌿 Intermediate |
| **CMakeLists.txt Issues** | `delivery_robot_perception/CMakeLists.txt` | 🌿 Intermediate |
| **Parameter Declarations** | `delivery_robot_perception/src/obstacle_detection.cpp` | 🌿 Intermediate |
| **Navigation Parameters** | `delivery_robot_navigation/config/nav2_params.yaml` | 🌳 Advanced |
| **TF Tree Issues** | Multiple files | 🌳 Advanced |
| **Action Server Timeouts** | `delivery_robot_core/src/delivery_manager.cpp` | 🌳 Advanced |

---

### Component Details:

| Component | Description | Status |
|-----------|-------------|--------|
| **delivery_robot_core** | Task management, delivery scheduling, robot state machine | ⚠️ Needs Testing |
| **delivery_robot_description** | URDF robot model, sensors, controllers | ✅ Working |
| **delivery_robot_navigation** | Nav2 configuration, maps, path planning | ⚠️ Needs Tuning |
| **delivery_robot_perception** | Obstacle detection, clustering, visualization | ❌ **BROKEN** |

---

## 🛠️ Features (When Fixed)

- ✅ **Complete Robot URDF** - 6-wheel differential drive with LiDAR and IMU
- ✅ **Navigation2 Integration** - Autonomous navigation with obstacle avoidance
- ✅ **Real-world Map Support** - Load and navigate using real maps
- ✅ **Obstacle Detection** - Laser scan processing and clustering (NEEDS FIXING)
- ✅ **Delivery Task Management** - Queue and execute delivery tasks
- ✅ **Production-ready Structure** - Modular packages with proper separation
- ✅ **RViz Visualization** - Complete visualization setup
- ✅ **Docker Support** - Containerized development environment
- ✅ **Simulation Ready** - Gazebo integration for testing

---

## 📦 Prerequisites

Before you begin, ensure you have the following installed:

### System Requirements
- **Ubuntu 22.04** (Jammy Jellyfish)
- **ROS2 Humble** (Desktop-Full installation recommended)
- **Python 3.10+**
- **Git**

### Install ROS2 Humble (if not already installed)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full
