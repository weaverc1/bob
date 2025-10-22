#!/usr/bin/env python3
"""
Micro-ROS ESP32 Integration Planning Script
Runs the Differential Drive Specialist to design a comprehensive micro-ROS integration plan
"""

from crew import AiMowerCrew
from crewai import Crew, Process
import sys


def main():
    print("=" * 80)
    print("BOB AUTONOMOUS MOWER - MICRO-ROS ESP32 INTEGRATION PLANNING")
    print("=" * 80)
    print()
    print("🎯 ARCHITECTURE REVIEW #1 QUICK WIN - HIGH IMPACT")
    print()
    print("The Differential Drive Specialist will design a micro-ROS integration plan to")
    print("replace the custom serial protocol with standardized ROS2 integration.")
    print()
    print("Current State:")
    print("  • ESP32 connects via custom serial @ 115200 baud")
    print("  • Custom protocol requires significant development overhead")
    print("  • Message type conversions needed")
    print()
    print("Target State (Micro-ROS):")
    print("  • Native ROS2 nodes running on ESP32")
    print("  • Standard message types (geometry_msgs, sensor_msgs)")
    print("  • Reduced development time (2-3 days)")
    print("  • Better ROS2 ecosystem integration")
    print()
    print("ESP32 Hardware Configuration:")
    print("  • ESP32-WROOM-32 Arduino board")
    print("  • L298N motor driver (GPIO 25,26,27,33)")
    print("  • BNO086 IMU on I2C (address 0x4B, 100Hz)")
    print("  • Wheel encoders (GPIO 32,35, 20 pulses/rev)")
    print("  • Differential drive (250x160mm, 67mm wheels)")
    print()
    print("The planning task will produce:")
    print("  ✓ Micro-ROS architecture design")
    print("  ✓ ESP32 firmware node structure")
    print("  ✓ Message type definitions")
    print("  ✓ Hardware interface design")
    print("  ✓ Build environment setup guide")
    print("  ✓ Testing strategy")
    print("  ✓ Migration plan from custom serial")
    print("  ✓ Code examples and snippets")
    print()
    print("=" * 80)
    print()

    response = input("Proceed with micro-ROS planning? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Planning cancelled.")
        sys.exit(0)

    print()
    print("Starting micro-ROS integration planning...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get the differential drive specialist and micro-ROS task
    specialist = crew_instance.differential_drive_specialist()
    microros_task = crew_instance.micro_ros_esp32_integration()

    # Create a focused crew for micro-ROS planning
    planning_crew = Crew(
        agents=[specialist],
        tasks=[microros_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute planning
    try:
        result = planning_crew.kickoff()

        print()
        print("=" * 80)
        print("MICRO-ROS INTEGRATION PLANNING COMPLETE!")
        print("=" * 80)
        print()
        print("Planning Result:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/micro_ros_integration_plan.md"
        with open(output_file, 'w') as f:
            f.write("# BOB Autonomous Mower - Micro-ROS ESP32 Integration Plan\n\n")
            f.write(str(result))

        print(f"Micro-ROS integration plan saved to: {output_file}")
        print()
        print("Next steps:")
        print("  1. Review the integration plan")
        print("  2. Set up micro-ROS build environment (PlatformIO or Arduino IDE)")
        print("  3. Install micro-ROS Arduino library")
        print("  4. Implement ESP32 firmware based on plan")
        print("  5. Test with ROS2 on Raspberry Pi")
        print("  6. Deploy to BOB hardware")
        print()
        print("Estimated implementation time: 2-3 days")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("PLANNING FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
