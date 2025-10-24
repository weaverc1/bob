#!/usr/bin/env python3
"""
Validate ESP32 to Raspberry Pi Pin Mapping
"""

from crew import AiMowerCrew
from crewai import Crew, Process
import sys

def main():
    print("=" * 80)
    print("ESP32 TO RASPBERRY PI PINOUT VALIDATION")
    print("=" * 80)
    print()
    print("Mission: Review ESP32 firmware and create complete Raspberry Pi GPIO mapping")
    print()
    print("The crew will:")
    print("  1. Extract ALL pin assignments from ESP32 code")
    print("  2. Map each pin to appropriate Raspberry Pi GPIO")
    print("  3. Validate no conflicts or missing functions")
    print("  4. Provide step-by-step wiring guide")
    print()
    print("ESP32 Code: esp32_code_for_review.ino")
    print("=" * 80)
    print()

    response = input("Proceed with pinout validation? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Validation cancelled.")
        sys.exit(0)

    print()
    print("Starting pinout validation...")
    print()

    crew_instance = AiMowerCrew()
    architect = crew_instance.system_architect()
    esp32_specialist = crew_instance.esp32_microros_specialist()
    
    validation_task = crew_instance.validate_esp32_to_pi_pinout()

    validation_crew = Crew(
        agents=[architect, esp32_specialist],
        tasks=[validation_task],
        process=Process.sequential,
        verbose=True
    )

    try:
        result = validation_crew.kickoff()

        print()
        print("=" * 80)
        print("PINOUT VALIDATION COMPLETE!")
        print("=" * 80)
        print()
        print(result)
        print()

        output_file = "output/esp32_to_pi_pinout.md"
        with open(output_file, 'w') as f:
            f.write("# ESP32 to Raspberry Pi GPIO Pin Mapping\n\n")
            f.write(str(result))

        print(f"✅ Pinout mapping saved to: {output_file}")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("VALIDATION FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
