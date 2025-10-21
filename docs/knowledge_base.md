# AI Mower Crew - Knowledge Base

This document will be populated by the System Architect agent with links to all relevant documentation, packages, and resources.

## Project Overview

**Project Name:** AI Mower - Autonomous Lawn Mower
**ROS Version:** ROS2 Jazzy
**Compute Platform:** Raspberry Pi 5
**OS:** Ubuntu 24.04

---

## ROS2 Core Documentation

### Official ROS2 Resources
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [ROS Index - Package Search](https://index.ros.org/)
- [ROS Discourse - Community Forum](https://discourse.ros.org/)

### Key Concepts
- [Understanding ROS2 Nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [Launch Files in ROS2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Parameters in ROS2](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)

---

## Navigation & SLAM

### Nav2 Stack
- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tutorials](https://navigation.ros.org/tutorials/index.html)
- [Nav2 GitHub Repository](https://github.com/ros-planning/navigation2)

### SLAM
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Cartographer ROS2](https://github.com/ros2/cartographer_ros)

---

## Control & Kinematics

### ros2_control
- [ros2_control Documentation](https://control.ros.org/jazzy/index.html)
- [ros2_control GitHub](https://github.com/ros-controls/ros2_control)
- [Differential Drive Controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)

### Odometry & Localization
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- TODO: Add odometry fusion packages

---

## Simulation

### Gazebo
- [Gazebo Documentation](https://gazebosim.org/docs)
- [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [URDF Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)

---

## Sensors

### Lidar
- TODO: Add Lidar driver packages (e.g., rplidar_ros, urg_node)

### IMU
- TODO: Add IMU driver packages (e.g., bno055_driver, imu_tools)

### Camera
- TODO: Add camera packages (e.g., v4l2_camera, image_pipeline)

---

## Hardware Resources

### Raspberry Pi 5
- [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/)
- [Ubuntu for Raspberry Pi](https://ubuntu.com/download/raspberry-pi)

### Motor Controllers
- TODO: Add motor controller documentation

---

## Safety Standards

- TODO: Add ISO standards for autonomous robots
- TODO: Add fail-safe design patterns
- TODO: Add emergency stop protocols

---

## Development Tools

### Build System
- [colcon - ROS2 Build Tool](https://colcon.readthedocs.io/)

### Testing
- [launch_testing](https://github.com/ros2/launch/tree/rolling/launch_testing)
- [pytest for ROS2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Python.html)

### Debugging
- [rqt Tools](https://docs.ros.org/en/jazzy/Concepts/About-RQt.html)
- [RViz2](https://github.com/ros2/rviz)

---

## Decision Log

### Architecture Decisions
- TODO: Document why specific packages were chosen
- TODO: Document architectural trade-offs

### Package Selection Rationale
- TODO: Document evaluation criteria
- TODO: Document alternatives considered

---

## Troubleshooting

### Common Issues
- TODO: Add common build errors and solutions
- TODO: Add runtime issues and fixes
- TODO: Add simulation-specific issues

---

## Contributing

### Code Style
- Follow [ROS2 Style Guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)

### Testing Requirements
- All new features must include unit tests
- Integration tests for multi-node systems
- Simulation tests before hardware deployment

---

*This knowledge base will be continuously updated as the project evolves.*
