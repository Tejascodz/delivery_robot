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



📚 Resources
Official Documentation

    ROS2 Humble Documentation

    Nav2 Documentation

    URDF Tutorial

    TF2 Documentation

ROS2 Learning

    ROS2 Tutorials

    ROS2 Style Guide

    ROS2 CLI Tools

Navigation

    Nav2 Parameters

    Costmap2D Documentation

    AMCL Documentation

C++ Resources

    ROS2 C++ Style Guide

    Modern C++ Features

🤝 Contributing

Please read CONTRIBUTING.md for details on our code of conduct and the process for submitting pull requests.
Contribution Tips

    Start Small: Begin with beginner-level bugs

    One Bug at a Time: Focus on fixing one issue per PR

    Test Your Fix: Ensure the system builds and runs

    Document Changes: Add comments explaining your fixes

    Be Patient: Maintainers will review your PR as soon as possible

Pull Request Checklist

    PR title clearly describes the fix

    Changes are limited to one bug/feature

    Code follows ROS2 style guidelines

    Comments added where necessary

    Builds without errors

    Tested with sample data

    Updated documentation if needed

📄 License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.
text

Copyright 2026 Tejascodz

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

🎉 Acknowledgments

    ROS2 Community - For creating an amazing robotics framework

    Nav2 Developers - For the powerful navigation stack

    Open Robotics - For advancing open-source robotics

    All Contributors - Who take time to fix these bugs and improve the project

⚠️ Disclaimer

IMPORTANT: This code contains intentional errors for educational purposes. Do not use in production without proper testing and fixing. The bugs are designed to teach common ROS2 pitfalls and debugging techniques.
<div align="center">
🌟 If you found this project helpful, please give it a star! 🌟

View Live Repository · Report Bug · Request Feature

Happy Bug Hunting! 🐛🔍

Remember: Every expert was once a beginner who fixed someone else's bugs.
</div> ```
