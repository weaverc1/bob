#!/usr/bin/env python3
"""
Micro-ROS Agent Installation Research Script
Has the crew research the proper way to install micro-ROS agent on Raspberry Pi 4
"""

from crew import AiMowerCrew
from crewai import Crew, Process, Task
import sys


def main():
    print("=" * 80)
    print("BOB AUTONOMOUS MOWER - MICRO-ROS AGENT INSTALLATION RESEARCH")
    print("=" * 80)
    print()
    print("🔍 RESEARCH REQUEST")
    print()
    print("Target System:")
    print("  • Platform: Raspberry Pi 4 Model B")
    print("  • OS: Ubuntu 24.04.3 LTS (Noble)")
    print("  • Architecture: aarch64 (ARM64)")
    print("  • ROS2 Distribution: Jazzy")
    print()
    print("Current Status:")
    print("  ✅ ROS2 Jazzy desktop installed successfully")
    print("  ✅ Navigation2, Nav2 Bringup, SLAM Toolbox, Robot Localization installed")
    print("  ❌ micro-ROS agent package not found in apt repositories")
    print()
    print("Error Encountered:")
    print("  Command: sudo apt install -y ros-jazzy-micro-ros-agent")
    print("  Result: E: Unable to locate package ros-jazzy-micro-ros-agent")
    print()
    print("Question:")
    print("  What is the correct way to install micro-ROS agent for ROS2 Jazzy")
    print("  on Ubuntu 24.04 ARM64 (Raspberry Pi 4)?")
    print()
    print("The System Architect will research:")
    print("  • Official micro-ROS agent installation methods")
    print("  • Package availability for Jazzy on ARM64")
    print("  • Alternative installation methods (snap, source build)")
    print("  • Step-by-step installation instructions")
    print()
    print("=" * 80)
    print()

    response = input("Proceed with research? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Research cancelled.")
        sys.exit(0)

    print()
    print("Starting micro-ROS agent installation research...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get the system architect
    architect = crew_instance.system_architect()

    # Create research task
    research_task = Task(
        description="""
        Research the correct installation method for micro-ROS agent on Raspberry Pi 4.

        SYSTEM DETAILS:
        - Platform: Raspberry Pi 4 Model B
        - OS: Ubuntu 24.04.3 LTS (Noble)
        - Architecture: aarch64 (ARM64)
        - ROS2 Distribution: Jazzy (latest)
        - Kernel: 6.8.0-1040-raspi

        PROBLEM:
        The package 'ros-jazzy-micro-ros-agent' is not available via apt:
        ```
        sudo apt install -y ros-jazzy-micro-ros-agent
        E: Unable to locate package ros-jazzy-micro-ros-agent
        ```

        SUCCESSFUL INSTALLATIONS:
        - ros-jazzy-desktop
        - ros-jazzy-navigation2
        - ros-jazzy-nav2-bringup
        - ros-jazzy-slam-toolbox
        - ros-jazzy-robot-localization

        RESEARCH OBJECTIVES:
        1. Determine if micro-ROS agent is available for ROS2 Jazzy on ARM64
        2. Find the correct package name (if different)
        3. Identify alternative installation methods:
           - Snap package
           - Source build from GitHub
           - Docker container
           - Pre-built binaries
        4. Provide step-by-step installation instructions
        5. Include verification commands
        6. Note any dependencies or prerequisites

        CONTEXT:
        We need micro-ROS agent to communicate with ESP32 running micro-ROS firmware.
        The ESP32 connects via USB serial at 115200 baud and publishes /odom, /imu topics
        and subscribes to /cmd_vel.

        Provide the most reliable and officially supported installation method.
        """,
        expected_output="""
        A comprehensive installation guide containing:
        - Official micro-ROS agent availability status for Jazzy ARM64
        - Correct package name (if available via apt)
        - Recommended installation method with full justification
        - Complete step-by-step installation commands
        - Dependency requirements
        - Verification commands to test installation
        - Expected output after successful installation
        - Troubleshooting tips
        - Alternative methods (ranked by preference)
        - Command to run micro-ROS agent with ESP32
        Format: Markdown with code blocks and clear sections
        """,
        agent=architect
    )

    # Create crew
    research_crew = Crew(
        agents=[architect],
        tasks=[research_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute
    try:
        result = research_crew.kickoff()

        print()
        print("=" * 80)
        print("MICRO-ROS AGENT INSTALLATION RESEARCH COMPLETE!")
        print("=" * 80)
        print()
        print("Research Results:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/microros_agent_installation.md"
        with open(output_file, 'w') as f:
            f.write("# Micro-ROS Agent Installation Guide for Raspberry Pi 4\n\n")
            f.write("## System Specifications\n\n")
            f.write("- Platform: Raspberry Pi 4 Model B\n")
            f.write("- OS: Ubuntu 24.04.3 LTS (Noble)\n")
            f.write("- Architecture: aarch64 (ARM64)\n")
            f.write("- ROS2: Jazzy\n\n")
            f.write("## Installation Guide\n\n")
            f.write(str(result))

        print(f"Installation guide saved to: {output_file}")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("RESEARCH FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
