#!/usr/bin/env python3
"""
ESP32 Firmware Code Review Script
Runs the Differential Drive Specialist to review the micro-ROS firmware implementation
"""

from crew import AiMowerCrew
from crewai import Crew, Process
import sys


def main():
    print("=" * 80)
    print("BOB AUTONOMOUS MOWER - ESP32 FIRMWARE CODE REVIEW")
    print("=" * 80)
    print()
    print("🔍 COMPREHENSIVE TECHNICAL REVIEW")
    print()
    print("The Differential Drive Specialist will conduct a thorough code review of")
    print("the ESP32 micro-ROS firmware implementation to ensure production readiness.")
    print()
    print("Firmware Under Review:")
    print("  📁 firmware/bob_microros_esp32/bob_microros_esp32.ino")
    print("  📄 firmware/INSTALLATION_INSTRUCTIONS.md")
    print("  📄 firmware/QUICK_START.md")
    print()
    print("Review Criteria:")
    print("  ✓ Code quality and maintainability")
    print("  ✓ Micro-ROS integration correctness")
    print("  ✓ Hardware interface implementation")
    print("  ✓ Motor control logic and kinematics")
    print("  ✓ Encoder ISR design and accuracy")
    print("  ✓ IMU integration and I2C communication")
    print("  ✓ Odometry algorithm and math")
    print("  ✓ Safety and error handling")
    print("  ✓ Performance and resource usage")
    print("  ✓ ROS2 message format compliance")
    print("  ✓ Installation guide completeness")
    print("  ✓ Bug identification")
    print("  ✓ Improvement recommendations")
    print("  ✓ Production readiness assessment")
    print()
    print("Expected Output:")
    print("  📊 Detailed code review report")
    print("  🐛 Bug identification (critical, major, minor)")
    print("  💡 Improvement recommendations")
    print("  ✅ Go/No-Go production decision")
    print()
    print("=" * 80)
    print()

    response = input("Proceed with firmware code review? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Code review cancelled.")
        sys.exit(0)

    print()
    print("Starting ESP32 firmware code review...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get the differential drive specialist and firmware review task
    specialist = crew_instance.differential_drive_specialist()
    review_task = crew_instance.esp32_firmware_review()

    # Create a focused crew for firmware review
    review_crew = Crew(
        agents=[specialist],
        tasks=[review_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute review
    try:
        result = review_crew.kickoff()

        print()
        print("=" * 80)
        print("ESP32 FIRMWARE CODE REVIEW COMPLETE!")
        print("=" * 80)
        print()
        print("Review Result:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/esp32_firmware_review.md"
        with open(output_file, 'w') as f:
            f.write("# BOB Autonomous Mower - ESP32 Firmware Code Review\n\n")
            f.write(str(result))

        print(f"Firmware code review saved to: {output_file}")
        print()
        print("Next steps:")
        print("  1. Review the detailed findings in output/esp32_firmware_review.md")
        print("  2. Address any critical or major issues identified")
        print("  3. Consider improvement recommendations")
        print("  4. Proceed with flashing if production-ready")
        print("  5. Re-review after making significant changes")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("CODE REVIEW FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
