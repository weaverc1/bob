#!/usr/bin/env python3
"""
Next Steps Request Script
Reports ESP32 installation success to the crew and requests next steps
"""

from crew import AiMowerCrew
from crewai import Crew, Process, Task
import sys


def main():
    print("=" * 80)
    print("BOB AUTONOMOUS MOWER - STATUS REPORT & NEXT STEPS REQUEST")
    print("=" * 80)
    print()
    print("📊 PROJECT STATUS UPDATE")
    print()
    print("✅ COMPLETED MILESTONES:")
    print("  1. Architecture Review - Comprehensive system evaluation")
    print("  2. Micro-ROS Integration Planning - Detailed design document")
    print("  3. ESP32 Firmware Development - v1.3 with safety features")
    print("  4. Firmware Code Review - AI crew validation")
    print("  5. Compilation Issues Fixed - ESP32 Arduino Core 3.x compatibility")
    print("  6. Runtime Errors Fixed - Watchdog and GPIO issues resolved")
    print("  7. ESP32 Firmware Flashed - Successfully running on hardware")
    print()
    print("🎯 CURRENT STATE:")
    print("  • ESP32 micro-ROS firmware v1.3 running")
    print("  • Serial output: micro-ROS handshake protocol (normal)")
    print("  • No error messages")
    print("  • Ready for ROS2 integration testing")
    print()
    print("🔧 HARDWARE STATUS:")
    print("  • ESP32-WROOM-32: Flashed with firmware v1.3")
    print("  • L298N Motor Driver: Connected (GPIO 25,26,27,33,32,14)")
    print("  • BNO086 IMU: Connected via I2C (address 0x4B)")
    print("  • Wheel Encoders: Connected (GPIO 34,35) - need external pullups")
    print("  • Raspberry Pi 4 (BOB): Awaiting ROS2 deployment")
    print()
    print("📝 PENDING ITEMS:")
    print("  • Deploy ROS2 Jazzy to BOB (Raspberry Pi)")
    print("  • Install micro-ROS agent on BOB")
    print("  • Test micro-ROS communication")
    print("  • Add external pullup resistors to encoders")
    print("  • Configure robot_localization EKF")
    print("  • Integration testing")
    print()
    print("Requesting next steps recommendation from System Architect...")
    print()
    print("=" * 80)
    print()

    response = input("Proceed with next steps request? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Request cancelled.")
        sys.exit(0)

    print()
    print("Requesting next steps from AI crew...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get the system architect
    architect = crew_instance.system_architect()

    # Create next steps task
    next_steps_task = Task(
        description="""
        The BOB autonomous mower project has successfully completed ESP32 firmware installation.

        COMPLETED WORK:
        1. ✅ Architecture review completed
        2. ✅ Micro-ROS integration plan created
        3. ✅ ESP32 firmware v1.3 developed with:
           - Micro-ROS integration (publishers, subscribers, executor)
           - Motor control (L298N driver)
           - Encoder reading with rollover protection
           - IMU integration (BNO086)
           - Odometry computation
           - Watchdog timer (3 second timeout)
           - cmd_vel timeout failsafe (1 second)
           - Velocity clamping and error checking
        4. ✅ Firmware code review by Differential Drive Specialist
        5. ✅ ESP32 Arduino Core 3.x compatibility fixes
        6. ✅ Runtime errors fixed (watchdog, GPIO pullups)
        7. ✅ Firmware successfully flashed to ESP32 hardware
        8. ✅ Serial monitor shows normal micro-ROS handshake protocol

        CURRENT STATE:
        - ESP32 running firmware v1.3, ready for ROS2 communication
        - Raspberry Pi 4 (BOB) has hardware connected but no ROS2 yet
        - Development machine has ROS2 Jazzy, Gazebo, testbot working

        HARDWARE CONFIGURATION:
        - Development Machine: Ubuntu 24.04, ROS2 Jazzy, Gazebo simulation
        - BOB (Raspberry Pi 4): Ubuntu 24.04, IP 192.168.86.62, hardware connected
        - ESP32: Firmware v1.3, connected to Pi via USB
        - Sensors: BNO086 IMU, SLLIDAR A1/A2, wheel encoders, GPS (future)
        - Motors: Differential drive with L298N driver

        CRITICAL PATH (from architecture review):
        1. Deploy ROS2 Jazzy to BOB ← NEXT STEP?
        2. Test ESP32-ROS2 communication via micro-ROS agent
        3. Implement robot_localization EKF (sensor fusion)
        4. Configure Nav2 and SLAM Toolbox
        5. Safety system implementation

        Based on the current state and critical path, provide a detailed recommendation for the
        next steps. Consider:
        - What should be done first?
        - What can be done in parallel?
        - What are the dependencies?
        - What are the risks?
        - What is the estimated timeline?
        - Should we deploy to BOB now or test locally first?

        Provide specific, actionable next steps with clear prioritization.
        """,
        expected_output="""
        A detailed next steps recommendation document containing:
        - Executive summary (immediate next action)
        - Recommended task sequence (prioritized)
        - Parallel work opportunities
        - Dependency analysis
        - Risk assessment for each step
        - Estimated timeline for each step
        - Detailed instructions for the immediate next step
        - Testing strategy
        - Success criteria for each step
        - Contingency plans
        Format: Markdown with clear sections and actionable items
        """,
        agent=architect
    )

    # Create crew
    next_steps_crew = Crew(
        agents=[architect],
        tasks=[next_steps_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute
    try:
        result = next_steps_crew.kickoff()

        print()
        print("=" * 80)
        print("NEXT STEPS RECOMMENDATION COMPLETE!")
        print("=" * 80)
        print()
        print("Recommendation:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/next_steps_recommendation.md"
        with open(output_file, 'w') as f:
            f.write("# BOB Autonomous Mower - Next Steps Recommendation\n\n")
            f.write("## Project Status\n\n")
            f.write("✅ ESP32 Firmware v1.3 Successfully Flashed\n")
            f.write("✅ Micro-ROS Integration Complete\n")
            f.write("✅ All Runtime Errors Fixed\n\n")
            f.write("## Recommendation\n\n")
            f.write(str(result))

        print(f"Next steps recommendation saved to: {output_file}")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("REQUEST FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
