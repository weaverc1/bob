#!/usr/bin/env python3
"""
ESP32 Boot Cycling Fix Script
Runs the crew in hierarchical mode to review and fix the ESP32 boot cycling issue
"""

from crew import AiMowerCrew
from crewai import Crew, Process
import sys


def main():
    print("=" * 80)
    print("BOB AUTONOMOUS MOWER - ESP32 BOOT CYCLING FIX")
    print("=" * 80)
    print()
    print("🚨 URGENT ISSUE: ESP32 Boot Cycling Every ~4 Seconds")
    print()
    print("Current Symptoms:")
    print("  ⚠️  ESP32 reconnecting continuously (delete_client/create_client pattern)")
    print("  ⚠️  Garbled bootloader output initially")
    print("  ⚠️  Topics visible but system unstable")
    print("  ⚠️  Connection: /dev/ttyUSB0 @ 115200 baud")
    print()
    print("The crew will:")
    print("  1. 🔍 Review firmware (differential_drive_specialist, esp32_microros_specialist)")
    print("  2. 🛠️  Generate fixed firmware (code_writer)")
    print("  3. 💾 Save fixed firmware to firmware/bob_microros_esp32/bob_microros_esp32_v1.4.ino")
    print()
    print("Crew Configuration:")
    print("  📋 Process: Hierarchical (round-robin collaboration)")
    print("  👥 All agents participate in review and implementation")
    print()
    print("=" * 80)
    print()

    response = input("Proceed with firmware fix? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Firmware fix cancelled.")
        sys.exit(0)

    print()
    print("Starting ESP32 firmware diagnosis and fix...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get specific agents for the task
    code_writer = crew_instance.code_writer()
    esp32_specialist = crew_instance.esp32_microros_specialist()
    drive_specialist = crew_instance.differential_drive_specialist()

    # Get the fix task
    fix_task = crew_instance.esp32_boot_cycling_fix()

    # Create specialized crew for firmware fix (sequential mode)
    crew = Crew(
        agents=[esp32_specialist, drive_specialist, code_writer],
        tasks=[fix_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute fix
    try:
        result = crew.kickoff()

        print()
        print("=" * 80)
        print("ESP32 BOOT CYCLING FIX COMPLETE!")
        print("=" * 80)
        print()
        print("Result:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/esp32_boot_cycling_fix.md"
        with open(output_file, 'w') as f:
            f.write("# BOB Autonomous Mower - ESP32 Boot Cycling Fix\n\n")
            f.write(str(result))

        print(f"✅ Fix documentation saved to: {output_file}")
        print()
        print("Next steps:")
        print("  1. Review the fixed firmware code")
        print("  2. Flash the new firmware to ESP32")
        print("  3. Monitor serial output for boot stability")
        print("  4. Verify topics are publishing correctly")
        print("  5. Test motor control via /cmd_vel")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("FIRMWARE FIX FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
