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
┌─────────────────────────────────────────────────────┐
│ Delivery Robot System │
├─────────────────────────────────────────────────────┤
│ ┌─────────────────┐ ┌─────────────────────┐ │
│ │ Core │ │ Navigation │ │
│ │ Management │──────│ Stack │ │
│ │ (delivery_robot_core) │ (delivery_robot_navigation)│ │
│ └─────────────────┘ └─────────────────────┘ │
│ │ │ │
│ ▼ ▼ │
│ ┌─────────────────┐ ┌─────────────────────┐ │
│ │ Description │ │ Perception │ │
│ │ URDF │ │ (BROKEN!) │ │
│ │(delivery_robot_description)│(delivery_robot_perception)│ │
│ └─────────────────┘ └─────────────────────┘ │
│ │
│ ┌─────────────────────────────────────────────┐ │
│ │ Communication Layer │ │
│ │ Topics: /scan, /odom, /cmd_vel, etc. │ │
│ └─────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
