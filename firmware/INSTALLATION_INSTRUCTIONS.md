# BOB ESP32 Micro-ROS Firmware - Complete Installation Guide

## Overview

This guide provides **explicit step-by-step instructions** to flash the BOB micro-ROS firmware onto your ESP32-WROOM-32 using Arduino IDE.

**Estimated Time**: 30-45 minutes (first time setup)

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Install Arduino IDE](#install-arduino-ide)
3. [Configure Arduino IDE for ESP32](#configure-arduino-ide-for-esp32)
4. [Install Micro-ROS Library](#install-micro-ros-library)
5. [Install Additional Libraries](#install-additional-libraries)
6. [Open and Configure Firmware](#open-and-configure-firmware)
7. [Connect ESP32 Hardware](#connect-esp32-hardware)
8. [Compile and Upload Firmware](#compile-and-upload-firmware)
9. [Setup Micro-ROS Agent on Raspberry Pi](#setup-micro-ros-agent-on-raspberry-pi)
10. [Testing and Verification](#testing-and-verification)
11. [Troubleshooting](#troubleshooting)

---

## Prerequisites

**Hardware Required:**
- ESP32-WROOM-32 (Arduino compatible board)
- USB cable (micro-USB or USB-C depending on your ESP32 board)
- Windows, Mac, or Linux computer
- BOB hardware connected (motors, encoders, IMU) - can flash without, test after

**Software Required:**
- Internet connection
- ~500MB free disk space

---

## 1. Install Arduino IDE

### Option A: Windows

1. Go to https://www.arduino.cc/en/software
2. Click **"Windows Win 10 and newer, 64 bits"**
3. Click **"JUST DOWNLOAD"** (or donate if you wish)
4. Run the downloaded `.exe` installer
5. Follow installation wizard, accept defaults
6. Launch Arduino IDE 2.x

### Option B: Linux (Ubuntu/Debian)

```bash
# Download latest Arduino IDE
cd ~/Downloads
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.2_Linux_64bit.AppImage

# Make executable
chmod +x arduino-ide_2.3.2_Linux_64bit.AppImage

# Run Arduino IDE
./arduino-ide_2.3.2_Linux_64bit.AppImage
```

### Option C: Mac

1. Go to https://www.arduino.cc/en/software
2. Click **"macOS"** (Intel or Apple Silicon)
3. Download and open the `.dmg` file
4. Drag Arduino IDE to Applications folder
5. Launch from Applications

---

## 2. Configure Arduino IDE for ESP32

### Step 1: Add ESP32 Board Manager URL

1. Open Arduino IDE
2. Click **File** → **Preferences** (or **Arduino IDE** → **Settings** on Mac)
3. Find **"Additional boards manager URLs"** field
4. Paste this URL:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
5. Click **OK**

### Step 2: Install ESP32 Board Package

1. Click **Tools** → **Board** → **Boards Manager...**
2. In search box, type: `esp32`
3. Find **"esp32 by Espressif Systems"**
4. Click **INSTALL** (this may take 5-10 minutes)
5. Wait for installation to complete
6. Close Boards Manager

### Step 3: Select ESP32 Board

1. Click **Tools** → **Board** → **esp32** → **ESP32 Dev Module**
   - If you have a specific board model, you can select it (e.g., "ESP32-WROOM-DA Module")
2. Leave other settings at defaults for now

---

## 3. Install Micro-ROS Library

### Method 1: Library Manager (Recommended)

1. Click **Sketch** → **Include Library** → **Manage Libraries...**
2. In search box, type: `micro_ros_arduino`
3. Find **"micro_ros_arduino"** by micro-ROS
4. Click **INSTALL**
5. Wait for installation (may take 2-3 minutes)
6. Close Library Manager

### Method 2: Manual Installation (If Method 1 Fails)

```bash
cd ~/Arduino/libraries/
git clone https://github.com/micro-ROS/micro_ros_arduino.git
```

Then restart Arduino IDE.

---

## 4. Install Additional Libraries

### Install Adafruit BNO08x Library (for IMU)

1. Click **Sketch** → **Include Library** → **Manage Libraries...**
2. Search: `Adafruit BNO08x`
3. Find **"Adafruit BNO08x"** by Adafruit
4. Click **INSTALL**
5. When prompted to install dependencies, click **INSTALL ALL**
   - This will install: Adafruit BusIO, Adafruit Unified Sensor

---

## 5. Open and Configure Firmware

### Step 1: Transfer Firmware to Your Computer

If you're working on the development machine, copy the firmware folder:

```bash
# Copy firmware to a location accessible from Windows/Arduino IDE
cp -r /home/ros2dev/ai_mower_crew/firmware/bob_microros_esp32 ~/Documents/Arduino/
```

**Or manually:**
1. Navigate to `/home/ros2dev/ai_mower_crew/firmware/bob_microros_esp32/`
2. Copy the entire folder to your `Documents/Arduino/` directory

### Step 2: Open Firmware in Arduino IDE

1. Click **File** → **Open...**
2. Navigate to `Documents/Arduino/bob_microros_esp32/`
3. Select `bob_microros_esp32.ino`
4. Click **Open**

The firmware should now be loaded in Arduino IDE.

### Step 3: Review Pin Configuration (Optional)

The firmware is pre-configured for your hardware. Check lines 31-40 if you need to verify:

```cpp
// Motor Driver L298N
#define MOTOR_LEFT_PWM    25
#define MOTOR_LEFT_DIR1   26
// ... etc
```

**No changes needed unless your wiring is different.**

---

## 6. Connect ESP32 Hardware

### Step 1: Connect ESP32 to Computer

1. Connect ESP32 to your computer via USB cable
2. Wait for drivers to install (Windows may take 1-2 minutes)

### Step 2: Select COM Port

#### Windows:
1. Open **Device Manager**
2. Expand **Ports (COM & LPT)**
3. Look for **"CP210x USB to UART Bridge"** or **"Silicon Labs CP210x"**
4. Note the COM port number (e.g., COM3, COM7)

In Arduino IDE:
1. Click **Tools** → **Port**
2. Select the COM port you identified (e.g., **COM3**)

#### Linux:
```bash
# List USB devices
ls -l /dev/ttyUSB*
# Or
ls -l /dev/ttyACM*

# Add your user to dialout group (one-time setup)
sudo usermod -a -G dialout $USER
# Log out and log back in
```

In Arduino IDE:
1. Click **Tools** → **Port**
2. Select `/dev/ttyUSB0` (or whichever port appeared)

#### Mac:
In Arduino IDE:
1. Click **Tools** → **Port**
2. Select `/dev/cu.SLAB_USBtoUART` or similar

---

## 7. Compile and Upload Firmware

### Step 1: Configure Upload Settings

Verify these settings in **Tools** menu:

- **Board**: "ESP32 Dev Module"
- **Upload Speed**: 921600
- **CPU Frequency**: 240MHz (WiFi/BT)
- **Flash Frequency**: 80MHz
- **Flash Mode**: QIO
- **Flash Size**: 4MB (32Mb)
- **Partition Scheme**: Default 4MB with spiffs
- **Core Debug Level**: None
- **Port**: Your COM port / ttyUSB0

### Step 2: Compile (Verify) Firmware

1. Click the **✓ (Verify)** button in top-left toolbar
2. Wait for compilation (first time may take 2-5 minutes)
3. Check output panel at bottom for any errors

**Expected output:**
```
Sketch uses XXXXX bytes (XX%) of program storage space.
Global variables use XXXXX bytes (XX%) of dynamic memory.
```

### Step 3: Upload to ESP32

1. Press and hold the **BOOT** button on your ESP32 board
2. While holding BOOT, click the **→ (Upload)** button in Arduino IDE
3. Keep holding BOOT until you see "Connecting..." in the output
4. Release BOOT button
5. Upload will proceed automatically

**Upload process takes ~30 seconds.**

**Expected output:**
```
Connecting.....
Writing at 0x00001000... (3 %)
...
Writing at 0x000f9000... (100 %)
Wrote 825120 bytes (489234 compressed) at 0x00010000 in 11.5 seconds
...
Hard resetting via RTS pin...
```

### Step 4: Verify Upload Success

1. After upload completes, ESP32 will automatically reset
2. Built-in LED should briefly flash, then turn off (indicating successful startup)
3. Click **Tools** → **Serial Monitor**
4. Set baud rate to **115200**
5. Press **EN** (reset) button on ESP32

**Expected output:**
```
BOB ESP32 Micro-ROS Node Ready!
```

---

## 8. Setup Micro-ROS Agent on Raspberry Pi

The ESP32 firmware needs a **micro-ROS agent** running on the Raspberry Pi to communicate with ROS2.

### Step 1: SSH to Raspberry Pi (BOB)

```bash
ssh bob@192.168.86.62
# Password: (your password)
```

### Step 2: Install Micro-ROS Agent

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install micro-ROS agent
sudo apt update
sudo apt install -y ros-jazzy-micro-ros-agent

# Verify installation
ros2 run micro_ros_agent --help
```

### Step 3: Connect ESP32 to Raspberry Pi

**Hardware connection:**
1. Connect ESP32 to Raspberry Pi via USB cable
2. ESP32 should power on (LED indicator)

**Identify USB port:**
```bash
# On Raspberry Pi, list USB devices
ls -l /dev/ttyUSB*
# Should show /dev/ttyUSB0 or /dev/ttyUSB1
```

### Step 4: Run Micro-ROS Agent

```bash
# Run agent on serial port
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Expected output:**
```
[1634567890.123456] info     | TermiosAgentLinux.cpp | init | running...
[1634567890.234567] info     | Root.cpp | create_client | create
[1634567890.345678] info     | SessionManager.hpp | establish_session | session established
```

---

## 9. Testing and Verification

### Terminal 1: Run Micro-ROS Agent (on Raspberry Pi)

```bash
ssh bob@192.168.86.62
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Keep this running!

### Terminal 2: List ROS2 Topics (on Raspberry Pi)

```bash
ssh bob@192.168.86.62
source /opt/ros/jazzy/setup.bash

# List active topics
ros2 topic list
```

**Expected output:**
```
/cmd_vel
/imu
/odom
/parameter_events
/rosout
```

### Terminal 3: Echo Odometry Data

```bash
ros2 topic echo /odom
```

You should see odometry messages being published at ~100Hz.

### Terminal 4: Echo IMU Data

```bash
ros2 topic echo /imu
```

You should see IMU messages with orientation, angular velocity, and acceleration.

### Terminal 5: Send Test Commands (Motor Control)

```bash
# Send forward command (0.1 m/s)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Motors should start moving!

# Send stop command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Send rotation command (0.5 rad/s)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

---

## 10. Troubleshooting

### Issue: Arduino IDE Can't Find ESP32 Board

**Solution:**
1. Verify ESP32 board package is installed (Section 2, Step 2)
2. Restart Arduino IDE
3. Check Board Manager URL is correct

### Issue: "Port Not Found" or "Access Denied"

**Windows:**
- Install CP210x USB driver: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

**Linux:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Issue: Upload Fails - "Connecting........_____....."

**Solution:**
1. Hold **BOOT** button on ESP32
2. Press **EN** (reset) button briefly
3. Release **EN**, keep holding **BOOT**
4. Click Upload in Arduino IDE
5. Release **BOOT** after "Connecting..."

### Issue: Compilation Errors - "micro_ros_arduino.h not found"

**Solution:**
1. Verify micro-ROS library is installed (Section 3)
2. Try manual installation method
3. Restart Arduino IDE

### Issue: "Adafruit_BNO08x.h not found"

**Solution:**
1. Install Adafruit BNO08x library (Section 4)
2. Install all dependencies when prompted

### Issue: ESP32 Resets/Crashes Continuously

**Possible causes:**
1. **IMU not connected**: Comment out IMU code temporarily
2. **Power issue**: Use powered USB hub
3. **Memory issue**: Check compilation output for memory usage

**Temporary IMU bypass:**
In `bob_microros_esp32.ino`, line ~335, comment out:
```cpp
// if (!setup_imu()) {
//   Serial.println("IMU initialization failed!");
// }
```

### Issue: Micro-ROS Agent "Session Not Established"

**Solution:**
1. Verify correct baud rate: `-b 115200`
2. Check USB cable (try different cable/port)
3. Press EN button on ESP32 to reset
4. Check Serial Monitor for "BOB ESP32 Micro-ROS Node Ready!"

### Issue: No IMU Data Published

**Check IMU wiring:**
- SDA → GPIO 21
- SCL → GPIO 22
- VCC → 3.3V
- GND → GND
- I2C address: 0x4B (check with i2cdetect on Arduino)

**Verify with test code:**
```cpp
// In setup(), add debugging:
Serial.println("Scanning I2C...");
Wire.begin(21, 22);
byte error = Wire.beginTransmission(0x4B);
Wire.endTransmission();
if (error == 0) {
  Serial.println("IMU found at 0x4B");
} else {
  Serial.println("IMU not found!");
}
```

### Issue: Motors Not Responding

**Check:**
1. L298N motor driver powered (separate 12V supply)
2. Motor driver connections match pin definitions
3. Enable pin on L298N (some modules have jumpers)
4. Test with simple PWM code first

---

## Summary Checklist

- [ ] Arduino IDE installed
- [ ] ESP32 board package installed
- [ ] Micro-ROS Arduino library installed
- [ ] Adafruit BNO08x library installed
- [ ] Firmware compiled successfully
- [ ] Firmware uploaded to ESP32
- [ ] ESP32 connected to Raspberry Pi via USB
- [ ] Micro-ROS agent running on Raspberry Pi
- [ ] ROS2 topics visible (`/odom`, `/imu`, `/cmd_vel`)
- [ ] Odometry data publishing
- [ ] IMU data publishing
- [ ] Motors respond to `/cmd_vel` commands

---

## Next Steps

Once firmware is working:

1. **Calibrate motor PWM mapping** - Adjust velocity-to-PWM conversion in `apply_motor_control()`
2. **Tune odometry** - Verify encoder counts match physical wheel movement
3. **Configure robot_localization EKF** - Fuse IMU + odometry + GPS
4. **Integrate with Nav2** - Enable autonomous navigation

---

## Support

If you encounter issues not covered here:

1. Check Serial Monitor output for error messages
2. Verify all hardware connections
3. Test components individually (motors, encoders, IMU separately)
4. Check ROS2 agent logs for connection issues

**Firmware version**: BOB Micro-ROS ESP32 v1.0
**Last updated**: 2025-10-22
