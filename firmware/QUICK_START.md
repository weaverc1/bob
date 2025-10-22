# BOB ESP32 Firmware - Quick Start Guide

## 🚀 Super Quick Setup (Experienced Users)

### 1. Arduino IDE Setup (5 min)

```
1. Install Arduino IDE 2.x from arduino.cc
2. File → Preferences → Additional boards manager URLs:
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
3. Tools → Board → Boards Manager → Install "esp32"
4. Sketch → Include Library → Manage Libraries:
   - Install "micro_ros_arduino"
   - Install "Adafruit BNO08x" (+ dependencies)
```

### 2. Upload Firmware (2 min)

```
1. File → Open → bob_microros_esp32.ino
2. Tools → Board → ESP32 Dev Module
3. Tools → Port → Select your COM/ttyUSB port
4. Hold BOOT button on ESP32
5. Click Upload (→)
6. Release BOOT when "Connecting..." appears
```

### 3. Run on Raspberry Pi (1 min)

```bash
# Terminal 1: Run micro-ROS agent
ssh bob@192.168.86.62
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2: Test topics
ros2 topic list
ros2 topic echo /odom

# Terminal 3: Test motors
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

---

## 📋 Pin Configuration Reference

```
Motors (L298N):
  Left:  PWM=25, DIR1=26, DIR2=27
  Right: PWM=33, DIR1=32, DIR2=14

Encoders:
  Left:  GPIO 34 (input-only)
  Right: GPIO 35 (input-only)

IMU (BNO086):
  I2C:   SDA=21, SCL=22
  Addr:  0x4B
  Rate:  100Hz
```

---

## 🔧 Arduino IDE Settings

```
Board: ESP32 Dev Module
Upload Speed: 921600
CPU Frequency: 240MHz
Flash Frequency: 80MHz
Flash Mode: QIO
Flash Size: 4MB
Partition Scheme: Default 4MB
Core Debug Level: None
```

---

## 📡 ROS2 Topics

```
Published:
  /odom → nav_msgs/Odometry (100Hz)
  /imu  → sensor_msgs/Imu (100Hz)

Subscribed:
  /cmd_vel ← geometry_msgs/Twist
```

---

## 🧪 Quick Tests

### Test 1: Check Serial Output
```
Tools → Serial Monitor (115200 baud)
Press EN button → Should show: "BOB ESP32 Micro-ROS Node Ready!"
```

### Test 2: Verify ROS2 Connection
```bash
ros2 topic list
# Should show: /odom, /imu, /cmd_vel
```

### Test 3: Check Odometry
```bash
ros2 topic hz /odom
# Should show: ~100 Hz
```

### Test 4: Test Motors
```bash
# Forward 0.1 m/s
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Rotate 0.5 rad/s
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## ⚠️ Common Issues

| Problem | Solution |
|---------|----------|
| **Upload fails** | Hold BOOT button, press EN, keep BOOT held, click Upload |
| **Port not found** | Windows: Install CP210x driver<br>Linux: `sudo usermod -a -G dialout $USER` |
| **Compilation error** | Check libraries installed (micro_ros_arduino, Adafruit_BNO08x) |
| **No ROS2 topics** | Check agent running, correct USB port, baud rate 115200 |
| **IMU not working** | Check I2C wiring (SDA=21, SCL=22, 3.3V, GND) |
| **Motors not moving** | Check L298N power supply (12V), enable pins/jumpers |

---

## 📦 File Locations

```
Firmware: /home/ros2dev/ai_mower_crew/firmware/bob_microros_esp32/
Full Guide: /home/ros2dev/ai_mower_crew/firmware/INSTALLATION_INSTRUCTIONS.md
Arduino Sketches: ~/Documents/Arduino/ (or ~/Arduino/)
```

---

## 🔗 Useful Commands

```bash
# Check USB devices (Linux)
ls -l /dev/ttyUSB*
dmesg | tail

# Install micro-ROS agent (Raspberry Pi)
sudo apt install ros-jazzy-micro-ros-agent

# Run agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Monitor all topics
ros2 topic echo /odom &
ros2 topic echo /imu &

# Record bag file for analysis
ros2 bag record /odom /imu /cmd_vel
```

---

## 🎯 Success Criteria

✅ Arduino IDE compiles without errors
✅ Upload completes successfully
✅ Serial Monitor shows "BOB ESP32 Micro-ROS Node Ready!"
✅ `ros2 topic list` shows /odom, /imu, /cmd_vel
✅ `ros2 topic echo /odom` shows live data
✅ Motors respond to /cmd_vel commands
✅ IMU publishes orientation data

---

**For detailed instructions, see [INSTALLATION_INSTRUCTIONS.md](INSTALLATION_INSTRUCTIONS.md)**
