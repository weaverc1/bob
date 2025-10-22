# BOB - Autonomous Lawn Mower

**AI Mower Crew**: A multi-agent CrewAI system for designing, developing, and deploying an autonomous lawn mower using ROS2 Jazzy.

[![GitHub](https://img.shields.io/badge/github-weaverc1/bob-blue)](https://github.com/weaverc1/bob)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green)](https://docs.ros.org/en/jazzy/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry_Pi_4-red)](https://www.raspberrypi.com/)
[![Architecture](https://img.shields.io/badge/Status-Architecture_Reviewed-success)](docs/)

## Overview

**BOB** (Build-Operate-Build) is an autonomous lawn mower project developed using an AI-driven approach. This project uses CrewAI to orchestrate **11 specialized AI agents** that collaborate to design and implement a complete autonomous lawn mowing system. Each agent brings expertise in specific domains such as system architecture, safety, navigation, control systems, simulation, testing, and remote deployment.

**GitHub Repository:** https://github.com/weaverc1/bob

### 🎯 Current Status (2025-10-21)

✅ **Architecture Reviewed & Validated** - The System Architect has completed a comprehensive review
✅ **Development Environment Ready** - ROS2 Jazzy, Gazebo simulation, testbot working
✅ **Hardware Documented** - Complete ESP32-based system specifications
✅ **Remote Deployment System** - SSH-based deployment to BOB (Raspberry Pi 4) ready
🔄 **Next:** Deploy ROS2 to BOB and implement Micro-ROS integration

See [Architecture Review](output/architecture_review.md) for detailed findings and recommendations.

### 🏆 Key Architecture Decisions

Based on comprehensive AI crew analysis:

- **✅ Dual-System Setup**: Development machine (simulation) + BOB (hardware)
- **✅ ROS2 Jazzy**: Chosen for both platforms (Ubuntu 24.04)
- **✅ Navigation Stack**: Nav2 + SLAM Toolbox + robot_localization
- **🔄 ESP32 Integration**: Switching from custom serial to **Micro-ROS** (recommended)
- **✅ Sensor Fusion**: Extended Kalman Filter (EKF) for IMU + encoders + LIDAR + GPS
- **✅ Safety Standard**: ISO 13849 PL d compliance target

## System Requirements

- **OS:** Ubuntu 24.04 (via WSL2 on Windows 11 or native)
- **ROS2 Distribution:** Jazzy
- **Python:** 3.12+
- **Compute Platform:** Raspberry Pi 5 (for deployment)
- **Development Platform:** Any system capable of running ROS2 Jazzy

## Project Structure

```
ai_mower_crew/
├── config/
│   ├── agents.yaml              # 10 specialized agent definitions
│   ├── tasks.yaml               # Task definitions for each agent
│   └── hardware_inventory.yaml  # Hardware specifications (to be filled)
├── tools/
│   └── github_search_tool.py    # ROS2 package discovery tool
├── simulation/
│   ├── urdf/                    # Robot URDF/Xacro models
│   ├── worlds/                  # Gazebo world files
│   ├── gazebo_plugins/          # Gazebo plugin configurations
│   └── launch/                  # Simulation launch files
├── launch/
│   ├── base_system.launch.py   # Core system launch
│   ├── nav_stack.launch.py     # Navigation stack launch
│   └── sensors_only.launch.py  # Sensor testing launch
├── test/
│   ├── sanity_check.test.py    # Basic ROS2 infrastructure tests
│   ├── imu_test.launch.py      # IMU validation
│   └── diff_drive_validation.launch.py  # Drivetrain tests
├── docs/
│   └── knowledge_base.md        # Centralized documentation
├── crew.py                      # CrewAI orchestration
├── main.py                      # Entry point
├── to_do.txt                    # Next tasks tracker
└── README.md                    # This file
```

## The 11 AI Agents

1. **System Architect** - Designs overall ROS2 system architecture, conducts architecture reviews
2. **Safety Engineer** - Identifies risks, implements fail-safes, ensures ISO 13849 PL d compliance
3. **Navigation Specialist** - Configures SLAM and Nav2 stack for outdoor autonomous navigation
4. **Differential Drive Specialist** - Handles drivetrain and odometry with encoder fusion
5. **ROS Code Hunter** - Finds and adapts existing ROS2 packages from GitHub
6. **Simulator** - Creates URDFs and Gazebo environments for realistic testing
7. **Test Specialist** - Designs incremental validation protocols
8. **The Realist** - Keeps project realistic within budget/time constraints
9. **ROS Infrastructure Builder** - Manages launch files and configuration
10. **Controller & Plugin Integrator** - Bridges simulation and hardware via ros2_control
11. **Remote Deployment Specialist** - Deploys and configures ROS2 on BOB via SSH (NEW!)

## Installation

### 1. Install Prerequisites

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Jazzy (if not already installed)
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop

# Source ROS2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install Python and tools
sudo apt install -y python3.12 python3.12-venv python3-pip
```

### 2. Set Up Python Virtual Environment

```bash
# Create virtual environment
python3.12 -m venv ~/crewai-env

# Activate virtual environment
source ~/crewai-env/bin/activate

# Install CrewAI
pip install --upgrade pip
pip install crewai crewai-tools
```

### 3. Navigate to Project

```bash
cd ~/ai_mower_crew
```

## Usage

### Running the Crew

Activate the virtual environment and run the crew:

```bash
source ~/crewai-env/bin/activate
cd ~/ai_mower_crew
python main.py
```

This will execute all tasks sequentially, with agents collaborating to produce:
- System architecture documentation
- Safety analysis
- Navigation stack configuration
- Differential drive design
- ROS2 package recommendations
- URDF models
- Gazebo worlds
- Testing protocols
- And more...

### Output

Results will be saved to `output/crew_results.md` and include:
- Markdown documentation
- Configuration files
- Code snippets
- Diagrams (Mermaid format)
- Package recommendations with URLs

### Advanced Usage

```bash
# Train the crew (improves performance over iterations)
python main.py train

# Replay a specific task
python main.py replay <task_id>

# Test mode
python main.py test

# Show help
python main.py help
```

## Configuration

### Hardware Inventory

Before running the crew, fill out [config/hardware_inventory.yaml](config/hardware_inventory.yaml) with your actual hardware specifications:

- Robot dimensions
- Sensors (Lidar, IMU, cameras, GPS)
- Motors and encoders
- Controllers
- Power system
- Safety components

### Customizing Agents

Edit [config/agents.yaml](config/agents.yaml) to modify agent roles, goals, or backstories.

### Customizing Tasks

Edit [config/tasks.yaml](config/tasks.yaml) to add, remove, or modify tasks.

## Next Steps

See [to_do.txt](to_do.txt) for immediate priorities:

1. **Complete hardware specification** in `config/hardware_inventory.yaml`
2. **Measure robot dimensions** and update URDF templates
3. **Set up GitHub integration** in `tools/github_search_tool.py`

## Development Workflow

### Phase 1: Design (Current Phase)
- Run CrewAI agents to generate architecture and design docs
- Review and validate recommendations
- Prioritize features based on constraints

### Phase 2: Simulation
- Complete URDF models with real dimensions
- Configure Gazebo plugins
- Test navigation and control in simulation

### Phase 3: Hardware Integration
- Set up Raspberry Pi 5
- Test components incrementally
- Validate sensor integration
- Test differential drive control

### Phase 4: Navigation & Autonomy
- Configure SLAM
- Tune Nav2 parameters
- Test autonomous operation
- Validate safety systems

## Testing

```bash
# Run sanity checks
cd ~/ai_mower_crew
python3 test/sanity_check.test.py

# Launch component tests (requires ROS2)
ros2 launch test/imu_test.launch.py
ros2 launch test/diff_drive_validation.launch.py
```

## Simulation

```bash
# Launch mower in Gazebo (after completing URDF and plugins)
ros2 launch simulation/launch/mower_gazebo.launch.py

# Launch testbot in indoor arena
ros2 launch simulation/launch/testbot_gazebo.launch.py
```

## Troubleshooting

### CrewAI Not Found
Ensure virtual environment is activated:
```bash
source ~/crewai-env/bin/activate
```

### ROS2 Command Not Found
Source ROS2 setup:
```bash
source /opt/ros/jazzy/setup.bash
```

### YAML Parse Errors
Validate YAML syntax:
```bash
python3 -c "import yaml; yaml.safe_load(open('config/agents.yaml'))"
```

## Contributing

This is a personal robotics project, but contributions and suggestions are welcome!

1. Document any hardware changes in `config/hardware_inventory.yaml`
2. Update `to_do.txt` as tasks are completed
3. Add learnings to `docs/knowledge_base.md`
4. Follow ROS2 coding standards

## Resources

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [CrewAI Documentation](https://docs.crewai.com/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Gazebo Documentation](https://gazebosim.org/docs)

## License

This project is for educational and personal use.

## Project Status

**Current Phase:** Initial Setup & Design
**Next Milestone:** Complete hardware specification and run first crew execution

---

**Created with CrewAI** - Multi-agent collaboration for autonomous robotics
