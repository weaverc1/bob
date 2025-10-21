# AI Mower Crew - Quick Start Guide

## Immediate Next Steps

### 1. Activate Virtual Environment
```bash
source ~/crewai-env/bin/activate
```

### 2. Navigate to Project
```bash
cd ~/ai_mower_crew
```

### 3. Review Next Tasks
```bash
cat to_do.txt
```

### 4. Fill Hardware Specifications
Edit the hardware inventory file:
```bash
nano config/hardware_inventory.yaml
```

Fill in:
- Robot dimensions (length, width, height, wheelbase, wheel radius)
- Sensor specifications (Lidar, IMU, camera models and interfaces)
- Motor and controller details
- Power system information

### 5. Run the Crew
Once hardware specs are ready:
```bash
python main.py
```

This will execute all 10 agents sequentially and generate:
- System architecture documentation
- Safety analysis
- Navigation configuration
- URDF designs
- Testing protocols
- ROS2 package recommendations

Results will be saved in `output/crew_results.md`

## Quick Commands

### Check Installation
```bash
# Verify ROS2
ros2 --help

# Verify Python environment
source ~/crewai-env/bin/activate
python --version  # Should show 3.12.x
pip list | grep crew  # Should show crewai packages
```

### Test Project
```bash
# Validate configuration files
python3 -c "import yaml; yaml.safe_load(open('config/agents.yaml'))"

# Test crew initialization
python3 -c "from crew import AiMowerCrew; c = AiMowerCrew(); print('✓ Crew ready!')"
```

### Run Specific Modes
```bash
# Normal execution
python main.py

# Training mode
python main.py train

# Help
python main.py help
```

## Project Structure Overview

```
ai_mower_crew/
├── config/          # Agent and task configurations + hardware specs
├── simulation/      # URDF models, Gazebo worlds, launch files
├── launch/          # ROS2 launch files for real hardware
├── test/            # Testing scripts and validation
├── tools/           # Custom tools (GitHub search, etc.)
├── docs/            # Documentation and knowledge base
├── main.py          # Run this to execute the crew
└── to_do.txt        # Your task tracker
```

## What Each Agent Does

1. **System Architect** - Designs the overall ROS2 system architecture
2. **Safety Engineer** - Plans fail-safes and emergency protocols
3. **Navigation Specialist** - Configures SLAM and Nav2
4. **Differential Drive Specialist** - Handles traction and odometry
5. **ROS Code Hunter** - Finds existing ROS2 packages to reuse
6. **Simulator** - Creates URDFs and Gazebo environments
7. **Test Specialist** - Plans incremental component testing
8. **The Realist** - Keeps scope realistic given budget/time
9. **ROS Infrastructure Builder** - Organizes launch files and configs
10. **Controller & Plugin Integrator** - Bridges sim and real hardware

## Immediate Priorities

Based on [to_do.txt](to_do.txt):

1. ✅ **DONE:** CrewAI project setup
2. ⏳ **NEXT:** Fill `config/hardware_inventory.yaml` with actual hardware
3. ⏳ **NEXT:** Measure/finalize robot dimensions
4. ⏳ **NEXT:** Set up GitHub integration for package discovery

## Getting Help

- See [README.md](README.md) for full documentation
- See [to_do.txt](to_do.txt) for task tracking
- See [docs/knowledge_base.md](docs/knowledge_base.md) for ROS2 resources

## Common Issues

**Issue:** `crewai` not found
**Solution:** Make sure virtual environment is activated:
```bash
source ~/crewai-env/bin/activate
```

**Issue:** YAML errors
**Solution:** Validate YAML syntax:
```bash
python3 -c "import yaml; yaml.safe_load(open('config/agents.yaml'))"
```

**Issue:** ROS2 command not found
**Solution:** Source ROS2:
```bash
source /opt/ros/jazzy/setup.bash
```

---

**Ready to build an autonomous lawn mower!** 🤖🌿
