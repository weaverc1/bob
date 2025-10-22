#!/usr/bin/env python3
"""
BOB Remote Deployment Script
Deploys ROS2 Jazzy to Raspberry Pi 4 via SSH
"""

from crew import AiMowerCrew
from crewai import Task
import sys


def main():
    print("=" * 80)
    print("BOB REMOTE DEPLOYMENT - AI Mower Crew")
    print("=" * 80)
    print()
    print("Target: Raspberry Pi 4 (BOB) at 192.168.86.62")
    print("Username: bob")
    print("Task: Deploy ROS2 Jazzy and configure system")
    print()
    print("The Remote Deployment Specialist will:")
    print("  1. Check system status (Ubuntu, disk space, USB devices)")
    print("  2. Clean up old ROS2 installations")
    print("  3. Install ROS2 Jazzy")
    print("  4. Install required packages (Nav2, SLAM Toolbox, etc.)")
    print("  5. Create workspace at /home/bob/bob_ws")
    print("  6. Configure udev rules for ESP32 and LIDAR")
    print("  7. Verify all installations")
    print()
    print("=" * 80)
    print()

    # Confirm with user
    response = input("Proceed with deployment? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Deployment cancelled.")
        sys.exit(0)

    print()
    print("Starting deployment...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get the deployment agent and task
    deployment_agent = crew_instance.remote_deployment_specialist()
    deployment_task = crew_instance.bob_remote_deployment()

    # Create a minimal crew with just the deployment task
    from crewai import Crew, Process

    deployment_crew = Crew(
        agents=[deployment_agent],
        tasks=[deployment_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute deployment
    try:
        result = deployment_crew.kickoff()

        print()
        print("=" * 80)
        print("DEPLOYMENT COMPLETE!")
        print("=" * 80)
        print()
        print("Deployment Result:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/bob_deployment_log.md"
        with open(output_file, 'w') as f:
            f.write("# BOB Deployment Log\n\n")
            f.write(str(result))

        print(f"Deployment log saved to: {output_file}")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("DEPLOYMENT FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
