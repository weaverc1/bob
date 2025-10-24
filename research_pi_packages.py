#!/usr/bin/env python3
"""
Research ROS2 Packages for Raspberry Pi GPIO Control
Find existing packages for motors, encoders, and IMU
"""

from crew import AiMowerCrew
from crewai import Crew, Process
import sys


def main():
    print("=" * 80)
    print("ROS2 PACKAGE RESEARCH FOR RASPBERRY PI")
    print("=" * 80)
    print()
    print("Mission: Find existing ROS2 packages for Raspberry Pi GPIO hardware")
    print()
    print("Components to Research:")
    print("  🔧 L298N Motor Driver (2 motors) - PWM + GPIO control")
    print("  📏 Wheel Encoders (2) - Digital pulse counting, odometry")
    print("  📡 BNO086 IMU - I2C 9-DOF sensor")
    print()
    print("The System Architect will search for:")
    print("  - Existing maintained ROS2 packages")
    print("  - Installation and configuration guides")
    print("  - Integration timelines")
    print("  - Recommendation: use packages OR build custom nodes")
    print()
    print("=" * 80)
    print()

    response = input("Proceed with package research? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Research cancelled.")
        sys.exit(0)

    print()
    print("Starting ROS2 package research...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get system architect
    architect = crew_instance.system_architect()

    # Get the research task
    research_task = crew_instance.research_ros2_pi_packages()

    # Create research crew
    research_crew = Crew(
        agents=[architect],
        tasks=[research_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute research
    try:
        result = research_crew.kickoff()

        print()
        print("=" * 80)
        print("ROS2 PACKAGE RESEARCH COMPLETE!")
        print("=" * 80)
        print()
        print("Findings:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/ros2_pi_packages_research.md"
        with open(output_file, 'w') as f:
            f.write("# ROS2 Packages for Raspberry Pi GPIO Control\n\n")
            f.write(str(result))

        print(f"✅ Research report saved to: {output_file}")
        print()
        print("Next steps:")
        print("  1. Review package recommendations")
        print("  2. Install recommended packages on Raspberry Pi")
        print("  3. Configure GPIO pin mappings")
        print("  4. Test each component individually")
        print("  5. Integrate and test full system")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("RESEARCH FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
