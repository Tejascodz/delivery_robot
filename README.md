# 🤖 Autonomous Delivery Robot - ROS2 Humble

[![ROS2](https://img.shields.io/badge/ROS2-Humble-34aec5)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Issues](https://img.shields.io/badge/Issues-Welcome-brightgreen)](https://github.com/yourusername/delivery_robot/issues)

## 🚀 A Production-Ready Autonomous Delivery Robot System

This project implements a world-class autonomous delivery robot using ROS2 Humble. The system includes complete robot description, navigation stack, perception modules, and delivery management - but there's a catch! 

## 🎯 **YOUR MISSION: Fix the Bugs!**

This repository contains a nearly complete delivery robot system, but there are intentional **bugs and errors** in the code that you need to fix. This is designed as a learning exercise for ROS2 developers.

### Current Issues to Fix:

1. **Header Guard Error** in `delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp`
   - The header file has syntax errors
   - Missing includes
   - Incomplete function declarations

2. **CMakeLists.txt Issues**
   - Missing dependencies
   - Incorrect target linking

3. **Runtime Errors**
   - Navigation parameters need tuning
   - TF tree issues
   - Action server timeouts

## 📋 System Architecture

┌─────────────────────────────────────┐
│ Delivery Robot System │
├─────────────────────────────────────┤
│ ┌─────────────┐ ┌───────────────┐ │
│ │ Core │ │ Navigation │ │
│ │ Management │ │ Stack │ │
│ └─────────────┘ └───────────────┘ │
│ ┌─────────────┐ ┌───────────────┐ │
│ │ Description │ │ Perception │ │
│ │ URDF │ │ (BROKEN) │ │
│ └─────────────┘ └───────────────┘ │
└─────────────────────────────────────┘
text


## 🛠️ Features (When Fixed)

- ✅ Complete robot URDF with sensors
- ✅ Navigation2 integration for autonomous navigation
- ✅ Real-world map support
- ✅ Obstacle detection and avoidance (NEEDS FIXING)
- ✅ Delivery task management
- ✅ Production-ready code structure
- ✅ RViz visualization
- ✅ Docker support

## 📦 Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo (optional)
- Python 3.10+

## 🚦 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/delivery_robot.git
cd delivery_robot

2. Install Dependencies
bash

chmod +x install_dependencies.sh
./install_dependencies.sh

3. Try to Build (This Will Fail!)
bash

chmod +x build.sh
./build.sh

Expected Output:
text

--- stderr: delivery_robot_perception
In file included from src/obstacle_detection.cpp:1:
include/delivery_robot_perception/obstacle_detection.hpp:47:5: error: ...

🐛 Known Bugs to Fix
Bug #1: Header Guard Mismatch

File: delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp
Line: 1
Error: Include guard doesn't match file path
Hint: Check the naming convention
Bug #2: Missing Includes

File: delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp
Error: Several required headers are missing
Hint: What ROS message types are being used?
Bug #3: Incomplete Function Declarations

File: delivery_robot_perception/include/delivery_robot_perception/obstacle_detection.hpp
Line: 52
Error: Function declared but not defined in cpp
Hint: Check the function signatures
Bug #4: CMake Linking Error

File: delivery_robot_perception/CMakeLists.txt
Error: Missing dependencies in target_link_libraries
Hint: What libraries does the perception node need?
Bug #5: Parameter Declaration

File: delivery_robot_perception/src/obstacle_detection.cpp
Error: Parameters declared but not initialized
Hint: Look for declare_parameter calls
🔧 How to Contribute

    Fork the repository

    Create a feature branch (git checkout -b fix/obstacle-detection)

    Fix one or more bugs

    Commit your changes (git commit -m 'Fix: obstacle detection header guard')

    Push to the branch (git push origin fix/obstacle-detection)

    Open a Pull Request

📝 Submission Guidelines

    Fix one bug per pull request

    Document your changes

    Add comments where necessary

    Test your fixes

    Update this README if needed

🏆 Challenge Levels
🌱 Beginner

    Fix header guard

    Add missing includes

    Fix parameter declarations

🌿 Intermediate

    Fix CMakeLists.txt

    Implement missing functions

    Fix runtime errors

🌳 Advanced

    Optimize clustering algorithm

    Add dynamic obstacle tracking

    Implement machine learning for object detection

📊 System Requirements

When fixed, the system should:

    Build without errors

    Launch without crashes

    Detect obstacles from laser scan

    Publish obstacle markers in RViz

    Navigate to goals while avoiding obstacles

🚀 Running the Fixed System

Once you've fixed the bugs:
bash

# Source ROS2
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Launch complete system
ros2 launch delivery_robot_core complete_system.launch.py

📁 Project Structure Explained
text

delivery_robot/
├── src/
│   ├── delivery_robot_core/           # Task management
│   ├── delivery_robot_description/     # Robot URDF
│   ├── delivery_robot_navigation/      # Nav2 config
│   └── delivery_robot_perception/      # BROKEN - NEEDS FIXING
├── install_dependencies.sh              # Setup script
├── build.sh                            # Build script
└── README.md                           # You are here

🆘 Getting Help

    Open an issue for questions

    Join our Discord (link below)

    Check ROS2 Humble documentation

    Review Nav2 tutorials

📚 Resources

    ROS2 Humble Documentation

    Nav2 Documentation

    URDF Tutorial

    ROS2 Style Guide

🤝 Contributing

Please read CONTRIBUTING.md for details on our code of conduct and the process for submitting pull requests.
📄 License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.
🎉 Acknowledgments

    ROS2 Community

    Nav2 Developers

    Open Robotics

    All contributors who fix these bugs!

⚠️ Disclaimer

This code contains intentional errors for educational purposes. Do not use in production without proper testing and fixing.

Happy Bug Hunting! 🐛🔍

Remember: Every expert was once a beginner who fixed someone else's bugs.
