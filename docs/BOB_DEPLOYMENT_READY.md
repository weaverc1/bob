# BOB Remote Deployment - READY TO LAUNCH

**Date:** 2025-10-21
**Target:** Raspberry Pi 4 Model B (BOB) at 192.168.86.62
**Status:** ✅ Ready for deployment

---

## Pre-Deployment Verification Complete

### SSH Connection: ✅ SUCCESSFUL
- **Host:** 192.168.86.62
- **Username:** bob
- **Password:** bob
- **Connection:** Verified working

### BOB System Information: ✅ VERIFIED
```
OS: Ubuntu 24.04.3 LTS (noble)
Kernel: 6.8.0-1017-raspi
Architecture: aarch64 (ARM 64-bit)
Platform: Raspberry Pi 4 Model B
Hostname: bob-mower
```

### Hardware Detected: ✅ CONFIRMED
**USB Devices:**
- VIA Labs USB Hub (2109:3431)
- Silicon Labs CP210x UART Bridge #1 (10c4:ea60) → /dev/ttyUSB0
- Silicon Labs CP210x UART Bridge #2 (10c4:ea60) → /dev/ttyUSB1
- Dell Keyboard (413c:2105)

**Device Mapping (to be verified):**
- **/dev/ttyUSB0** - Likely ESP32 (motor controller, encoders, IMU)
- **/dev/ttyUSB1** - Likely SLLIDAR

**Hardware Connected (as reported by user):**
- ✅ ESP32 microcontroller
  - L298N motor driver
  - BNO086 IMU (I2C)
  - Wheel encoders (GPIO 32, 35)
  - DC motors (left/right)
- ✅ SLLIDAR A1/A2 (360° 2D LIDAR)
- ✅ Motor driver (L298N)
- ✅ DC motors with encoders

---

## AI Crew Deployment Setup

### New Agent Created: Remote Deployment Specialist
**Role:** Remote Deployment & System Configuration Engineer

**Tools Available:**
- SSH Command Executor
- SSH File Transfer (SFTP)

**Expertise:**
- Remote robotics deployment
- ROS2 on ARM platforms (Raspberry Pi, Jetson)
- System administration via SSH
- udev rules and device management
- Package management and dependency resolution

### Deployment Task: bob_remote_deployment

The crew will execute the following steps:

**1. System Check**
- Verify Ubuntu version
- Check existing ROS2 installations
- Check disk space
- List USB devices

**2. Cleanup**
- Remove old ROS2 installations
- Clean up old workspaces
- Update system packages

**3. ROS2 Jazzy Installation**
- Add ROS2 Jazzy apt repository
- Install ROS2 Jazzy base
- Install development tools
- Configure environment variables

**4. Package Installation**
- ros-jazzy-navigation2 (Nav2 stack)
- ros-jazzy-slam-toolbox
- ros-jazzy-robot-localization
- ros-jazzy-ros2-control
- ros-jazzy-ros2-controllers
- ros-jazzy-ros-gz (Gazebo)
- ros-jazzy-rqt, ros-jazzy-rviz2
- ros-jazzy-imu-tools
- Additional utilities

**5. Workspace Creation**
- Create /home/bob/bob_ws
- Set up colcon build environment
- Copy testbot_sim package
- Build workspace

**6. udev Rules**
- Create /dev/ttyESP32 (persistent name for ESP32)
- Create /dev/ttyLIDAR (persistent name for SLLIDAR)
- Set proper permissions (dialout group)
- Reload udev rules

**7. ESP32 Bridge Setup**
- Create serial communication package
- Configure 115200 baud serial bridge
- Set up ROS2 topics (cmd_vel, odom, imu)

**8. Sensor Integration**
- Install sllidar_ros2 driver
- Configure robot_localization (EKF)
- Test sensor data streams

**9. System Configuration**
- Add ROS2 environment sourcing to ~/.bashrc
- Configure logging
- Network settings verification

**10. Final Verification**
- Test `ros2 pkg list`
- Verify USB device connections
- Check sensor data (LIDAR scan, IMU)
- Verify workspace builds

---

## How to Run the Deployment

### Method 1: Run Deployment Script (Recommended)

```bash
cd ~/ai_mower_crew
source ~/crewai-env/bin/activate
python deploy_bob.py
```

This will:
- Display deployment plan
- Ask for confirmation
- Run the Remote Deployment Specialist
- Save deployment log to `output/bob_deployment_log.md`

### Method 2: Run Full Crew (All Tasks)

If you want to run ALL crew tasks including deployment:

```bash
cd ~/ai_mower_crew
source ~/crewai-env/bin/activate
python main.py
```

**Note:** This will run all 13 tasks (12 original + 1 deployment), which takes ~35-40 minutes.

---

## Expected Deployment Time

- **System check & cleanup:** 2-3 minutes
- **ROS2 Jazzy installation:** 10-15 minutes
- **Package installation:** 15-20 minutes
- **Workspace setup:** 3-5 minutes
- **udev rules & configuration:** 2-3 minutes
- **Verification:** 2-3 minutes

**Total:** 35-50 minutes (depends on internet speed)

---

## Post-Deployment Verification

After deployment completes, verify manually via SSH:

```bash
ssh bob@192.168.86.62
# Password: bob
```

**Check ROS2 installation:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep nav2
ros2 pkg list | grep slam
```

**Check USB devices:**
```bash
ls -la /dev/ttyESP32 /dev/ttyLIDAR
```

**Check workspace:**
```bash
ls ~/bob_ws/src
source ~/bob_ws/install/setup.bash
```

**Test LIDAR (if available):**
```bash
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py serial_port:=/dev/ttyLIDAR
# In another terminal:
ros2 topic echo /scan
```

---

## Configuration Files

**BOB Configuration:** [config/bob_config.yaml](../config/bob_config.yaml)
- Connection details
- Hardware inventory
- Deployment task list

**SSH Tools:** [tools/ssh_tool.py](../tools/ssh_tool.py)
- SSH command executor
- File transfer capability

**Agents:** [config/agents.yaml](../config/agents.yaml)
- Remote Deployment Specialist added

**Tasks:** [config/tasks.yaml](../config/tasks.yaml)
- bob_remote_deployment task added

---

## Troubleshooting

**If SSH fails:**
```bash
# Test connection manually
ssh bob@192.168.86.62

# If key verification fails
ssh-keygen -R 192.168.86.62
```

**If deployment hangs:**
- Check BOB's internet connection
- Verify disk space: `df -h /`
- Check for running apt processes: `ps aux | grep apt`

**If packages fail to install:**
- Update package lists: `sudo apt update`
- Check ROS2 repository: `apt-cache policy | grep ros`

**If udev rules don't work:**
- Check device IDs: `udevadm info -a -n /dev/ttyUSB0`
- Reload udev: `sudo udevadm control --reload-rules && sudo udevadm trigger`
- Check dialout group: `groups bob`

---

## Cost Estimate

**AI Crew deployment task:**
- Model: GPT-4o-mini
- Estimated API calls: 20-30
- Estimated tokens: 100,000-150,000
- **Estimated cost: $0.03-$0.05**

Very cost-effective for automated remote deployment!

---

## Next Steps After Deployment

1. **Verify all sensors** (ESP32, LIDAR, IMU)
2. **Test motor control** via ESP32
3. **Run SLAM in indoor environment**
4. **Test navigation stack** (Nav2)
5. **Calibrate odometry** (wheel encoders + IMU)
6. **Field testing** (outdoor mowing)

---

## Ready to Deploy!

Everything is configured and tested. Run the deployment script when you're ready:

```bash
cd ~/ai_mower_crew
source ~/crewai-env/bin/activate
python deploy_bob.py
```

The Remote Deployment Specialist will take care of everything! 🤖

---

**End of Deployment Readiness Document**
