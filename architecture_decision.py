#!/usr/bin/env python3
"""
ESP32 vs Raspberry Pi Architecture Decision
Ask the crew for architectural recommendation
"""

from crew import AiMowerCrew
from crewai import Crew, Process
import sys


def main():
    print("=" * 80)
    print("ARCHITECTURAL DECISION: ESP32 vs RASPBERRY PI")
    print("=" * 80)
    print()
    print("Question: Should we continue with ESP32 + micro-ROS or move to Pi direct control?")
    print()
    print("Current Situation:")
    print("  ⚠️  ESP32 boot cycling every ~4 seconds")
    print("  ✅ v1.4 firmware created with fixes (not yet tested)")
    print("  🤔 Decision needed: Continue debugging OR change architecture")
    print()
    print("The System Architect will evaluate:")
    print("  1. Technical feasibility of both options")
    print("  2. Risk assessment and time estimates")
    print("  3. Performance and maintainability implications")
    print("  4. Clear recommendation with GPIO pinout if recommending Pi")
    print()
    print("=" * 80)
    print()

    response = input("Proceed with architectural analysis? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Analysis cancelled.")
        sys.exit(0)

    print()
    print("Starting architectural decision analysis...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get system architect and supporting agents
    architect = crew_instance.system_architect()
    realist = crew_instance.realist()
    esp32_specialist = crew_instance.esp32_microros_specialist()

    # Get the decision task
    decision_task = crew_instance.esp32_vs_pi_architecture_decision()

    # Create specialized crew for this decision
    decision_crew = Crew(
        agents=[architect, esp32_specialist, realist],
        tasks=[decision_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute analysis
    try:
        result = decision_crew.kickoff()

        print()
        print("=" * 80)
        print("ARCHITECTURAL DECISION ANALYSIS COMPLETE!")
        print("=" * 80)
        print()
        print("Recommendation:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/architecture_decision.md"
        with open(output_file, 'w') as f:
            f.write("# ESP32 vs Raspberry Pi Architecture Decision\n\n")
            f.write(str(result))

        print(f"✅ Decision document saved to: {output_file}")
        print()
        print("Next steps:")
        print("  1. Review the recommendation")
        print("  2. If ESP32: Test v1.4 firmware on hardware")
        print("  3. If Pi: Follow GPIO pinout and implementation roadmap")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("ANALYSIS FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
