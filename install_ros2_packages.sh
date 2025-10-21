#!/bin/bash
# Install Recommended ROS2 Packages for BOB Autonomous Mower
# Based on AI Mower Crew recommendations

set -e  # Exit on error

echo "================================================================================"
echo "Installing Recommended ROS2 Packages for BOB"
echo "================================================================================"
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash

echo "📦 Updating package lists..."
sudo apt update

echo ""
echo "================================================================================"
echo "Installing Navigation Stack (Nav2)"
echo "================================================================================"
sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-msgs

echo ""
echo "================================================================================"
echo "Installing SLAM Toolbox"
echo "================================================================================"
sudo apt install -y \
    ros-jazzy-slam-toolbox

echo ""
echo "================================================================================"
echo "Installing Robot Localization (Sensor Fusion)"
echo "================================================================================"
sudo apt install -y \
    ros-jazzy-robot-localization

echo ""
echo "================================================================================"
echo "Installing ros2_control Framework"
echo "================================================================================"
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager

echo ""
echo "================================================================================"
echo "Installing Gazebo (gz) Integration for ROS2 Jazzy"
echo "================================================================================"
echo "Note: ROS2 Jazzy uses Gazebo (gz) instead of classic Gazebo"
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces

echo ""
echo "================================================================================"
echo "Installing Additional Utilities"
echo "================================================================================"
sudo apt install -y \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro

echo ""
echo "================================================================================"
echo "Installing Sensor Packages"
echo "================================================================================"
sudo apt install -y \
    ros-jazzy-imu-tools \
    ros-jazzy-diagnostic-updater

echo ""
echo "================================================================================"
echo "Verifying Installations"
echo "================================================================================"

packages=(
    "ros-jazzy-navigation2"
    "ros-jazzy-slam-toolbox"
    "ros-jazzy-robot-localization"
    "ros-jazzy-ros2-control"
    "ros-jazzy-ros-gz"
)

echo ""
echo "Checking installed packages:"
for pkg in "${packages[@]}"; do
    if dpkg -l | grep -q "^ii.*$pkg"; then
        echo "  ✅ $pkg"
    else
        echo "  ❌ $pkg - NOT INSTALLED"
    fi
done

echo ""
echo "================================================================================"
echo "Installation Summary"
echo "================================================================================"
echo ""
echo "✅ Nav2 Navigation Stack"
echo "✅ SLAM Toolbox"
echo "✅ robot_localization (Sensor Fusion)"
echo "✅ ros2_control Framework"
echo "✅ Gazebo Integration"
echo "✅ RViz2 & RQT Tools"
echo "✅ IMU Tools"
echo ""
echo "================================================================================"
echo "Installation Complete!"
echo "================================================================================"
echo ""
echo "Next steps:"
echo "  1. Source ROS2: source /opt/ros/jazzy/setup.bash"
echo "  2. Test Nav2: ros2 pkg list | grep nav2"
echo "  3. Test SLAM: ros2 pkg list | grep slam"
echo "  4. Review crew results: cat ~/ai_mower_crew/output/crew_results.md"
echo ""
echo "All recommended packages from the AI Mower Crew are now installed!"
echo ""
