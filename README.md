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

## 📋 System Architecture


---

## 🚦 Getting Started

### 1. Clone the Repository

```bash
# Create workspace
mkdir -p ~/delivery_robot_ws/src
cd ~/delivery_robot_ws/src

# Clone repository
git clone https://github.com/Tejascodz/delivery_robot.git
cd ~/delivery_robot_ws
```

### 2. Try Building (This Will Fail!)

```bash
colcon build --symlink-install
```

Expected outcome: Build fails with errors. **This is intentional!** Your mission is to fix these errors.

### 3. Source the Workspace

```bash
source install/setup.bash
```

---

## 🐛 Known Bugs to Fix

### 🐞 Beginner Level

#### Bug #1: Header Guard Error
**File:** `delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp`

The header guard is incorrectly named. Fix it to match the file path convention.

#### Bug #2: Missing Includes
**File:** `delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp`

Missing necessary ROS2 includes for message types and node utilities.

#### Bug #3: Incomplete Function Declaration
**File:** `delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp`

The `clusterPoints()` function declaration is missing parameters.

### 🌿 Intermediate Level

#### Bug #4: CMakeLists.txt Issues
**File:** `delivery_robot_perception/CMakeLists.txt`

- Missing dependency declarations
- Incorrect include directories
- Missing library installation targets

#### Bug #5: Parameter Declaration
**File:** `delivery_robot_perception/src/obstacle_detection.cpp`

The node tries to declare parameters without proper error handling or using the correct ROS2 parameter API.

### 🌳 Advanced Level

#### Bug #6: Navigation Parameters
**File:** `delivery_robot_navigation/config/nav2_params.yaml`

- Incorrect parameter names for Nav2 stack
- Missing required parameters
- Invalid range values

#### Bug #7: TF Tree Issues
**Files:** Multiple files across packages

The transformation tree has incorrect frame IDs and missing transforms between robot base and sensor frames.

#### Bug #8: Action Server Timeouts
**File:** `delivery_robot_core/src/delivery_manager.cpp`

Action server timeouts are too short, causing delivery tasks to fail during complex maneuvers.

---

## 🔧 How to Contribute

### Step-by-Step Contribution Guide

1. **Fork the Repository**
   - Click the 'Fork' button on GitHub

2. **Clone Your Fork**
   ```bash
   git clone https://github.com/YOUR_USERNAME/delivery_robot.git
   cd delivery_robot
   ```

3. **Create a Branch**
   ```bash
   git checkout -b fix/bug-name
   ```

4. **Fix One Bug**
   - Focus on fixing just one issue at a time
   - Test your fix thoroughly
   - Add comments explaining your changes

5. **Commit Your Changes**
   ```bash
   git add .
   git commit -m "Fix: Description of bug fixed"
   ```

6. **Push and Create PR**
   ```bash
   git push origin fix/bug-name
   ```
   - Go to GitHub and create a Pull Request

---

## 🏆 Challenge Levels

### Level 1: Novice Debugger 🌱
- Fix header guards and missing includes
- Get the perception package to build
- **Reward**: First successful build!

### Level 2: Intermediate Debugger 🌿
- Fix CMakeLists.txt and parameter declarations
- Get the obstacle detection node to run
- **Reward**: See obstacle markers in RViz!

### Level 3: Expert Debugger 🌳
- Fix TF tree and navigation parameters
- Complete a full delivery mission
- **Reward**: Robot successfully navigates to delivery point!

### Level 4: ROS2 Master 🚀
- Optimize all systems
- Add new features
- **Reward**: Your name in contributors list!

---

## 📊 System Requirements

### Hardware Requirements
- **CPU**: 4+ cores recommended
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 20GB free space
- **GPU**: Optional (for simulation)

### Software Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Build Tools**: colcon, CMake 3.8+
- **Simulation**: Gazebo (optional)

---

## 🚀 Running the Fixed System

Once you've fixed all bugs, you can run the complete system:

### Terminal 1: Start the Robot Core
```bash
source install/setup.bash
ros2 launch delivery_robot_core delivery_system.launch.py
```

### Terminal 2: Start Navigation
```bash
source install/setup.bash
ros2 launch delivery_robot_navigation navigation.launch.py map:=/path/to/your/map.yaml
```

### Terminal 3: Send Delivery Task
```bash
source install/setup.bash
ros2 run delivery_robot_core send_delivery --x 2.5 --y -1.8
```

### Terminal 4: RViz Visualization
```bash
source install/setup.bash
ros2 run rviz2 rviz2 -d src/delivery_robot/delivery_robot_description/rviz/delivery_robot.rviz
```

---

## 📁 Project Structure

```
delivery_robot/
├── delivery_robot_core/
│   ├── config/              # Core configuration files
│   ├── include/             # Header files
│   ├── src/                 # Source files
│   ├── launch/              # Launch files
│   ├── actions/             # Action definitions
│   ├── CMakeLists.txt       # Build configuration (with bugs)
│   └── package.xml          # Package manifest
│
├── delivery_robot_description/
│   ├── urdf/                # Robot URDF models
│   ├── meshes/              # 3D mesh files
│   ├── launch/              # Display launch files
│   ├── rviz/                 # RViz configuration
│   ├── CMakeLists.txt       # Build configuration
│   └── package.xml          # Package manifest
│
├── delivery_robot_navigation/
│   ├── config/              # Nav2 parameters (with bugs)
│   ├── maps/                # Map files
│   ├── launch/              # Navigation launch files
│   ├── params/              # Additional parameters
│   ├── CMakeLists.txt       # Build configuration
│   └── package.xml          # Package manifest
│
├── delivery_robot_perception/
│   ├── include/             # Header files (with bugs)
│   ├── src/                 # Source files (with bugs)
│   ├── launch/              # Perception launch files
│   ├── config/              # Perception parameters
│   ├── CMakeLists.txt       # Build configuration (with bugs)
│   └── package.xml          # Package manifest
│
└── delivery_robot_bringup/
    ├── launch/              # Main system launch files
    ├── config/              # Global configuration
    ├── CMakeLists.txt       # Build configuration
    └── package.xml          # Package manifest
```

---

## 📚 Resources

### Official Documentation
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

### ROS2 Learning
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2 Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Guide.html)
- [ROS2 CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)

### Navigation
- [Nav2 Parameters](https://navigation.ros.org/configuration/index.html)
- [Costmap2D Documentation](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)
- [AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

### C++ Resources
- [ROS2 C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Guide-C++.html)
- [Modern C++ Features](https://en.cppreference.com/w/cpp)

---

## 🤝 Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and the process for submitting pull requests.

### Contribution Tips
- **Start Small**: Begin with beginner-level bugs
- **One Bug at a Time**: Focus on fixing one issue per PR
- **Test Your Fix**: Ensure the system builds and runs
- **Document Changes**: Add comments explaining your fixes
- **Be Patient**: Maintainers will review your PR as soon as possible

### Pull Request Checklist
- [ ] PR title clearly describes the fix
- [ ] Changes are limited to one bug/feature
- [ ] Code follows ROS2 style guidelines
- [ ] Comments added where necessary
- [ ] Builds without errors
- [ ] Tested with sample data
- [ ] Updated documentation if needed

---

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

```
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
```

---

## 🎉 Acknowledgments

- **ROS2 Community** - For creating an amazing robotics framework
- **Nav2 Developers** - For the powerful navigation stack
- **Open Robotics** - For advancing open-source robotics
- **All Contributors** - Who take time to fix these bugs and improve the project

---

## ⚠️ Disclaimer

**IMPORTANT**: This code contains intentional errors for educational purposes. Do not use in production without proper testing and fixing. The bugs are designed to teach common ROS2 pitfalls and debugging techniques.

---

<div align="center">

### 🌟 If you found this project helpful, please give it a star! 🌟

[View Live Repository](https://github.com/Tejascodz/delivery_robot) · [Report Bug](https://github.com/Tejascodz/delivery_robot/issues) · [Request Feature](https://github.com/Tejascodz/delivery_robot/issues)

**Happy Bug Hunting!** 🐛🔍

*Remember: Every expert was once a beginner who fixed someone else's bugs.*

</div>
```
