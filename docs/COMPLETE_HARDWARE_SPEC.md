# BOB Autonomous Mower - Complete Hardware Specification

**Project:** BOB (Autonomous Lawn Mower)
**Platform:** ROS2 Humble on Raspberry Pi + ESP32 Microcontroller
**Date:** 2025-10-21
**Status:** Hardware inventory complete, ready for AI Crew analysis

---

## Executive Summary

This document provides the complete, actual hardware inventory for the BOB autonomous lawn mower project. The system uses a **dual-compute architecture**:
- **Raspberry Pi** (Model 4/5) running ROS2 Humble for high-level navigation and planning
- **ESP32 microcontroller** handling real-time motor control, encoders, and IMU

All hardware listed below is **currently available and specified** with actual GPIO pins, communication protocols, and calibrated parameters.

---

## 1. COMPUTE ARCHITECTURE

### 1.1 Main Compute - Raspberry Pi
- **Model:** Raspberry Pi 4 or 5
- **Hostname:** bob-mower
- **User:** bob
- **OS:** Ubuntu (ROS2 Humble compatible)
- **ROS2 Distribution:** Humble Hawksbill
- **Role:** High-level navigation, SLAM, path planning, sensor fusion

### 1.2 Microcontroller - ESP32
- **Model:** ESP32 Development Board
- **Framework:** Arduino
- **Communication:** USB Serial @ 115200 baud
- **Device Path:** /dev/ttyESP32
- **USB Chipset:** Silicon Labs CP210x (USB ID: 10c4:ea60)
- **Role:** Real-time motor control, encoder counting, IMU interface

**I2C Configuration:**
- SDA: GPIO 21
- SCL: GPIO 22

**PWM Configuration:**
- Resolution: 8-bit (0-255)
- Frequency: 10,000 Hz
- Working range: 170-255 PWM
- Optimal range: 220-255 PWM
- Minimum threshold: 170 (below this = no movement)

---

## 2. MOTOR SYSTEM

### 2.1 Motor Driver - L298N Dual H-Bridge
- **Model:** L298N
- **Channels:** 2 (left and right motors)
- **Control:** PWM speed + GPIO direction
- **Voltage:** Compatible with system (6-60V capable)

**GPIO Pin Assignments:**

| Function | GPIO Pin |
|----------|----------|
| Left Motor IN1 (Direction) | 27 |
| Left Motor IN2 (Direction) | 26 |
| Left Motor ENA (PWM Speed) | 14 |
| Right Motor IN3 (Direction) | 25 |
| Right Motor IN4 (Direction) | 33 |
| Right Motor ENB (PWM Speed) | 12 |

**Forward Direction Logic:**
- Left Motor: IN1=HIGH, IN2=LOW
- Right Motor: IN3=LOW, IN4=HIGH

### 2.2 Drive Motors (2x)
- **Type:** DC Geared Motors with Integrated Encoders
- **Estimated Power:** 350W each
- **Maximum RPM:** 200
- **Tested Speed:** 250 PWM (out of 255)
- **Encoder Type:** Optical rotary with hall effect
- **Pulses per Revolution:** 20

---

## 3. WHEEL ENCODERS

### 3.1 Specifications
- **Type:** Optical rotary encoders with hall effect
- **Count:** 2 (left and right wheels)
- **Holes per Revolution:** 20
- **Pulses per Meter:** 413
- **Pulses per 360° Rotation:** 206
- **Meters per Pulse:** 0.00334m

### 3.2 GPIO Connections
- **Left Encoder:** GPIO 32 (with internal pullup)
- **Right Encoder:** GPIO 35 (with internal pullup)

### 3.3 Signal Processing
- **Trigger:** Rising edge interrupt
- **Debounce:** 1ms
- **Update Rate:** 100 Hz (10ms interval)

### 3.4 Calibration Data
- **Wheel Diameter:** 67mm (radius: 33.5mm)
- **Wheel Circumference:** 0.2105m (π × 0.067m)
- **Wheel Separation:** 135mm (track width)
- **Ticks per Revolution:** 20

---

## 4. SENSORS

### 4.1 IMU - SparkFun BNO086 (BNO08x)
- **Model:** BNO086 9-axis IMU
- **Interface:** I2C
- **I2C Address:** 0x4B
- **I2C Pins:** SDA=21, SCL=22
- **Update Rate:** 100 Hz (10ms interval)

**Capabilities:**
- Gyro-integrated rotation vector
- Linear acceleration
- Angular velocity
- Stability classifier

**Output Data:**
- Quaternion: (i, j, k, real)
- Angular Velocity: (x, y, z) in rad/s
- Linear Acceleration: (x, y, z) in m/s²
- Stability Status: Boolean

**Sensor Ranges:**
- Accelerometer: ±16g (160 m/s²)
- Gyroscope: ±2000 deg/s (35 rad/s)

**Mount Location:** 200mm behind base_link center, 25mm above base
**Frame ID:** imu_link
**Mass:** 0.01 kg
**ROS Driver:** bno08x_driver (to be developed/integrated)

### 4.2 LIDAR - SLAMTEC SLLIDAR (A1/A2)
- **Model:** SLLIDAR A1 or A2
- **Interface:** Serial UART
- **Device Path:** /dev/ttyLIDAR
- **USB Chipset:** Silicon Labs CP210x (USB ID: 10c4:ea60)
- **Baudrate:** 1,000,000 (A1/A2) or 256,000 (A3)

**Specifications:**
- **Scan Angle:** 360°
- **Update Frequency:** 10 Hz
- **Maximum Range:** 8.0 meters
- **Angle Compensation:** Enabled

**Physical:**
- **Dimensions:** Ø70mm × 40mm height (cylindrical)
- **Mount Location:** 100mm behind base_link center, 46mm above base
- **Frame ID:** laser_frame
- **Mass:** 0.1 kg

**ROS Driver:** sllidar_ros2

### 4.3 Camera - Logitech C270 Webcam
- **Model:** Logitech C270
- **Interface:** USB Video Class (UVC)
- **Device Path:** /dev/video_robot (or /dev/video0)
- **USB ID:** 046d:0825

**Specifications:**
- **Default Resolution:** 640×480
- **Maximum Resolution:** 1280×720
- **Frame Rate:** 30 FPS
- **Pixel Format:** YUYV/MJPEG

**Features:**
- Auto white balance
- Auto exposure
- Manual focus

**Adjustable Parameters:**
- Brightness: 128 (default)
- Contrast: 128
- Saturation: 128
- Gain: 100

**ROS Driver:** usb_cam
**Future Use:** Vision processing, object detection

### 4.4 GPS Module
- **Model:** Generic GPS with PL2303 Serial Adapter
- **Interface:** USB-Serial
- **USB Chipset:** Prolific PL2303 (USB ID: 067b:2303)
- **Status:** Physically connected, not yet integrated
- **Intended Use:** Absolute positioning for lawn coverage tracking
- **ROS Driver:** TBD (nmea_navsat_driver or similar)

---

## 5. ROBOT PHYSICAL PARAMETERS (CALIBRATED)

### 5.1 Base Dimensions
- **Length:** 250mm (0.250m)
- **Width:** 160mm (0.160m)
- **Height:** 50mm (0.050m)
- **Mass:** 0.875 kg (excluding battery/electronics)

### 5.2 Wheel Configuration
- **Type:** Differential Drive (2-wheel + caster)
- **Drive Wheel Diameter:** 67mm (radius: 33.5mm)
- **Drive Wheel Width:** 20mm
- **Wheel Separation:** 135mm (track width)
- **Drive Wheel Mass:** 0.1 kg each
- **Caster Diameter:** 30mm (radius: 15mm)
- **Caster Mass:** 0.05 kg
- **Caster Position:** 180mm behind base_link center, -35mm below base

### 5.3 Mass Distribution
- **Total Robot Mass:** ~1.125 kg
  - Base: 0.875 kg
  - Left Wheel: 0.1 kg
  - Right Wheel: 0.1 kg
  - Caster: 0.05 kg
- **Center of Mass Offset:** -90mm from front
- **Note:** Battery and electronics mass not included in above

---

## 6. CONTROL INPUT

### 6.1 Game Controller - 8bitdo Pro2
- **Type:** USB Game Controller
- **Interface:** USB (via Pygame joystick library)
- **Purpose:** Manual teleoperation

**Control Mapping:**
- **Forward/Backward:** Left stick vertical (axis 1)
- **Turning:** Right stick horizontal (axis 2)
- **Deadzone:** 1.5% (0.015)

---

## 7. POWER SYSTEM

### 7.1 Current Status
- **Battery Type:** TBD
- **Voltage Range:** 6-60V capable (based on L298N motor controller)
- **Capacity:** TBD
- **Voltage Regulators:**
  - 5V output for Raspberry Pi: Required
  - 12V output for motors: TBD

### 7.2 Future Considerations
- Automated charging dock integration
- Battery voltage monitoring
- Power management system

---

## 8. SAFETY SYSTEMS

### 8.1 Required Safety Components
- **Emergency Stop:** TBD (physical button or wireless kill switch)
- **Blade Guard:** Enabled
- **Blade Interlock:** Enabled (safety-critical)
- **Safety Standard:** ISO 13849 PL d (recommended for autonomous systems)

### 8.2 Safety Components - TBD
- Bump sensors
- Tilt sensor (with maximum angle threshold)
- Blade motor safety interlock

---

## 9. USB INFRASTRUCTURE

### 9.1 USB Hub
- **Model:** VIA Labs USB Hub
- **USB ID:** 2109:3431
- **Purpose:** Manages multiple USB devices

### 9.2 Serial Adapters
1. **CP210x UART Bridge (ESP32)**
   - USB ID: 10c4:ea60
   - Purpose: ESP32 connection

2. **CP210x UART Bridge (LIDAR)**
   - USB ID: 10c4:ea60
   - Purpose: SLLIDAR connection

3. **PL2303 USB-Serial (GPS)**
   - USB ID: 067b:2303
   - Purpose: GPS module connection

---

## 10. INTEGRATION ARCHITECTURE

### 10.1 Hardware Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    RASPBERRY PI (ROS2 Humble)               │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────┐  │
│  │   Nav2       │  │  SLAM        │  │  robot_         │  │
│  │   Stack      │  │  Toolbox     │  │  localization   │  │
│  └──────────────┘  └──────────────┘  └─────────────────┘  │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │           ROS2 Control & Sensor Fusion               │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────┬───────────────────┬───────────────┬───────────┘
              │                   │               │
       ┌──────▼──────┐   ┌────────▼─────────┐   │
       │   ESP32     │   │   SLLIDAR        │   │
       │   (Serial)  │   │   (USB-Serial)   │   │
       │  115200 bps │   │   1Mbps          │   │
       └──────┬──────┘   └──────────────────┘   │
              │                            ┌─────▼──────┐
    ┌─────────┴──────────┐                │  Logitech  │
    │                    │                │   C270     │
┌───▼────┐        ┌──────▼──────┐        │  (USB UVC) │
│ L298N  │        │   BNO086    │        └────────────┘
│ Motor  │        │     IMU     │
│ Driver │        │   (I2C)     │
└───┬────┘        └─────────────┘
    │
┌───▼────────────────────────┐
│  DC Motors + Encoders      │
│  (Left & Right Wheels)     │
└────────────────────────────┘
```

### 10.2 Communication Summary

| Component | Connection | Protocol | Baudrate/Rate |
|-----------|------------|----------|---------------|
| ESP32 → RPi | USB Serial | UART | 115200 bps |
| BNO086 IMU → ESP32 | I2C | I2C | 100 Hz |
| Encoders → ESP32 | GPIO | Digital | 100 Hz |
| Motors ← L298N ← ESP32 | GPIO + PWM | PWM | 10 kHz |
| SLLIDAR → RPi | USB Serial | UART | 1 Mbps |
| Camera → RPi | USB | UVC | 30 FPS |
| GPS → RPi | USB Serial | UART/NMEA | TBD |

---

## 11. KEY CHALLENGES FOR AI CREW

The AI Mower Crew should address the following critical integration challenges:

### 11.1 ESP32-ROS2 Bridge Design
**Challenge:** Design communication protocol between ESP32 and ROS2
**Options:**
- micro-ROS (full ROS2 on ESP32)
- Custom serial protocol (JSON, binary, or other)
- rosserial-equivalent for ROS2

**Requirements:**
- Low latency for motor commands
- Reliable encoder data transmission
- IMU data streaming
- Bidirectional command/status

### 11.2 Real-Time Motor Control Coordination
**Challenge:** Coordinate differential drive control with encoder feedback
**Requirements:**
- PID control loops for velocity control
- Odometry calculation from encoders
- Motor command safety limits
- Deadman switch integration

### 11.3 Sensor Fusion Architecture
**Challenge:** Fuse IMU, encoders, and LIDAR for accurate localization
**Requirements:**
- robot_localization EKF configuration
- IMU orientation quaternion → TF transforms
- Encoder odometry → wheel velocities
- LIDAR scan matching for drift correction

### 11.4 Safety System Integration
**Challenge:** Implement ISO 13849 PL d safety architecture
**Requirements:**
- Blade motor interlock (cannot run without safety conditions)
- Emergency stop integration (immediate motor cutoff)
- Tilt sensor monitoring (prevent operation on slopes)
- Watchdog timer (detect communication loss)

### 11.5 Navigation Stack Configuration for Outdoor Mowing
**Challenge:** Configure Nav2 for autonomous lawn coverage
**Requirements:**
- Coverage path planning algorithm
- Obstacle avoidance with LIDAR
- GPS waypoint integration (future)
- Return-to-dock behavior

### 11.6 Power Management & Battery Monitoring
**Challenge:** Monitor battery and prevent brownout
**Requirements:**
- Battery voltage monitoring (ADC on ESP32 or RPi)
- Low-battery return-to-dock behavior
- Power consumption estimation
- Charging dock detection

### 11.7 GPS Integration for Absolute Positioning
**Challenge:** Integrate GPS for lawn coverage mapping
**Requirements:**
- NMEA sentence parsing
- GPS → map frame transformation
- Coverage map persistence
- Overlap prevention in mowing pattern

### 11.8 Controller Input Handling
**Challenge:** Implement teleoperation with 8bitdo Pro2
**Requirements:**
- Joystick axis mapping with deadzone
- Mode switching (autonomous ↔ manual)
- Safety button for blade enable
- Emergency stop button

---

## 12. TESTBOT TO FULL-SIZE MOWER SCALING

### 12.1 Current Testbot (Indoor Platform)
- **Dimensions:** 250mm × 160mm × 50mm
- **Mass:** ~1.1 kg
- **Motors:** 350W DC geared @ 200 RPM
- **Purpose:** Algorithm validation, sensor integration testing

### 12.2 Full-Size Mower (Outdoor Platform) - TBD
- **Estimated Scale Factor:** 3-4× larger
- **Estimated Dimensions:** ~750-1000mm × 500-640mm
- **Estimated Mass:** 20-40 kg (including battery)
- **Motors:** Higher power (1-2 kW per wheel)
- **Blade Motor:** 1-2 kW brushless (safety-critical)

### 12.3 Shared Architecture
Both platforms use:
- Same sensor suite (IMU, LIDAR, Camera, GPS)
- Same ROS2 software stack
- Same ESP32-based motor control architecture
- Same safety protocols (scaled appropriately)

---

## 13. RECOMMENDED ROS2 PACKAGES

Based on hardware inventory, the following ROS2 packages are critical:

| Package | Purpose |
|---------|---------|
| navigation2 | Nav2 navigation stack |
| slam-toolbox | SLAM implementation |
| robot-localization | Sensor fusion (EKF) |
| ros2-control | Hardware abstraction layer |
| ros2-controllers | diff_drive_controller |
| gazebo-ros-pkgs | Simulation environment |
| gazebo-ros2-control | Simulated hardware interface |
| sllidar_ros2 | LIDAR driver |
| usb_cam | Camera driver |
| nmea_navsat_driver | GPS driver (future) |
| rqt | Visualization and debugging |
| rviz2 | 3D visualization |
| plotjuggler | Data plotting |

---

## 14. NEXT STEPS

1. **Run AI Mower Crew Analysis** - Let the 10-agent crew analyze this hardware specification
2. **ESP32-ROS2 Bridge Design** - Crew should recommend micro-ROS vs custom protocol
3. **Safety Architecture Design** - Crew should design ISO 13849 PL d compliance
4. **Navigation Configuration** - Crew should configure Nav2 for outdoor mowing
5. **Simulation Setup** - Create Gazebo world with accurate robot model
6. **Hardware Integration Testing** - Validate each sensor and actuator
7. **Field Testing Protocol** - Develop safe outdoor testing procedures

---

**End of Hardware Specification Document**
