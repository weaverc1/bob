#!/usr/bin/env python3
"""
System Architecture Review Script
Runs the System Architect to review the entire BOB autonomous mower system
"""

from crew import AiMowerCrew
from crewai import Crew, Process
import sys


def main():
    print("=" * 80)
    print("BOB AUTONOMOUS MOWER - SYSTEM ARCHITECTURE REVIEW")
    print("=" * 80)
    print()
    print("The System Architect will comprehensively review:")
    print("  • Development machine setup (Ubuntu 24.04, ROS2 Jazzy, Gazebo)")
    print("  • BOB Raspberry Pi 4 configuration (Ubuntu 24.04, ROS2 Jazzy)")
    print("  • Hardware specifications (ESP32, BNO086, SLLIDAR, motors, encoders)")
    print("  • ESP32-ROS2 bridge architecture")
    print("  • ROS2 package selections (Nav2, SLAM Toolbox, robot_localization)")
    print("  • Sensor fusion strategy (IMU + encoders + LIDAR + GPS)")
    print("  • Navigation stack configuration")
    print("  • Safety system architecture (ISO 13849 PL d)")
    print("  • Simulation-to-hardware workflow")
    print("  • Budget and resource constraints")
    print("  • Best practice compliance")
    print("  • Risk analysis and mitigation")
    print()
    print("The review will identify:")
    print("  ✓ Strengths of current architecture")
    print("  ⚠ Weaknesses and risks")
    print("  → Quick wins for improvement")
    print("  → Long-term recommendations")
    print()
    print("=" * 80)
    print()

    response = input("Proceed with architecture review? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Review cancelled.")
        sys.exit(0)

    print()
    print("Starting architecture review...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get the system architect and review task
    architect = crew_instance.system_architect()
    review_task = crew_instance.system_architecture_review()

    # Create a focused crew for the review
    review_crew = Crew(
        agents=[architect],
        tasks=[review_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute review
    try:
        result = review_crew.kickoff()

        print()
        print("=" * 80)
        print("ARCHITECTURE REVIEW COMPLETE!")
        print("=" * 80)
        print()
        print("Review Result:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/architecture_review.md"
        with open(output_file, 'w') as f:
            f.write("# BOB Autonomous Mower - System Architecture Review\n\n")
            f.write(str(result))

        print(f"Architecture review saved to: {output_file}")
        print()
        print("Next steps:")
        print("  1. Review the recommendations")
        print("  2. Update documentation based on findings")
        print("  3. Implement quick wins")
        print("  4. Plan long-term improvements")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("REVIEW FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
