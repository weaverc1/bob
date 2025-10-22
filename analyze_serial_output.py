#!/usr/bin/env python3
"""
ESP32 Serial Output Analysis Script
Analyzes ESP32 serial monitor output and provides debugging recommendations
"""

from crew import AiMowerCrew
from crewai import Crew, Process, Task
import sys


SERIAL_OUTPUT = """
Backtrace: 0x40089bdf:0x3ffd2cc0 0x40188081:0x3ffd2ce0 0x4008db53:0x3ffd2d00 0x4008cb89:0x3ffd2d20

ELF file SHA256: 9d42789bd

Rebooting...
ets Jul 29 2019 12:21:46

rst:0xc (SW_CPU_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0030,len:4980
load:0x40078000,len:16612
load:0x40080400,len:3480
entry 0x400805b4
E (25) task_wdt: esp_task_wdt_init(517): TWDT already initialized
Watchdog timer initialized
E (26) gpio: gpio_pullup_en(78): GPIO number error (input-only pad has no internal PU)
E (30) gpio: gpio_pullup_en(78): GPIO number error (input-only pad has no internal PU)
~�XRCE^��;��\\Q~�XRCE^��;��\\Q
"""


def main():
    print("=" * 80)
    print("BOB AUTONOMOUS MOWER - ESP32 SERIAL OUTPUT ANALYSIS")
    print("=" * 80)
    print()
    print("🔍 ANALYZING SERIAL MONITOR OUTPUT")
    print()
    print("The Differential Drive Specialist will analyze the ESP32 serial output")
    print("and provide debugging recommendations.")
    print()
    print("Serial Output:")
    print("-" * 80)
    print(SERIAL_OUTPUT)
    print("-" * 80)
    print()
    print("Firmware Version: v1.2")
    print("Firmware Location: firmware/bob_microros_esp32/bob_microros_esp32.ino")
    print()
    print("=" * 80)
    print()

    response = input("Proceed with serial output analysis? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Analysis cancelled.")
        sys.exit(0)

    print()
    print("Starting ESP32 serial output analysis...")
    print()

    # Create crew instance
    crew_instance = AiMowerCrew()

    # Get the differential drive specialist
    specialist = crew_instance.differential_drive_specialist()

    # Create custom analysis task
    analysis_task = Task(
        description=f"""
        Analyze the ESP32 serial monitor output from the BOB autonomous mower firmware v1.2.

        Serial Output:
        {SERIAL_OUTPUT}

        Firmware Details:
        - Version: v1.2
        - Location: firmware/bob_microros_esp32/bob_microros_esp32.ino
        - ESP32-WROOM-32 Arduino board
        - ESP32 Arduino Core 3.x
        - Hardware: L298N motor driver, BNO086 IMU, wheel encoders on GPIO 34,35

        Identify all errors and issues in the serial output:
        1. Boot sequence analysis
        2. Watchdog timer errors
        3. GPIO errors (input-only pad internal pullup)
        4. Garbled micro-ROS output
        5. Root cause analysis
        6. Impact assessment

        Provide specific fixes for each issue with code snippets and line numbers.
        """,
        expected_output="""
        A detailed serial output analysis containing:
        - Executive summary of all errors found
        - Boot sequence analysis (reset reason, boot mode)
        - Error 1: TWDT already initialized (line number, cause, fix)
        - Error 2: GPIO pullup errors on input-only pads (which GPIOs, cause, fix)
        - Error 3: Garbled micro-ROS output (cause, fix)
        - Root cause analysis for each error
        - Code fixes with specific line numbers
        - Testing recommendations
        - Expected serial output after fixes
        Format: Detailed Markdown report with code snippets
        """,
        agent=specialist
    )

    # Create analysis crew
    analysis_crew = Crew(
        agents=[specialist],
        tasks=[analysis_task],
        process=Process.sequential,
        verbose=True
    )

    # Execute analysis
    try:
        result = analysis_crew.kickoff()

        print()
        print("=" * 80)
        print("ESP32 SERIAL OUTPUT ANALYSIS COMPLETE!")
        print("=" * 80)
        print()
        print("Analysis Result:")
        print("-" * 80)
        print(result)
        print("-" * 80)
        print()

        # Save result to file
        output_file = "output/esp32_serial_analysis.md"
        with open(output_file, 'w') as f:
            f.write("# BOB Autonomous Mower - ESP32 Serial Output Analysis\n\n")
            f.write("## Serial Output\n\n")
            f.write("```\n")
            f.write(SERIAL_OUTPUT)
            f.write("```\n\n")
            f.write("## Analysis\n\n")
            f.write(str(result))

        print(f"Serial output analysis saved to: {output_file}")
        print()
        print("Next steps:")
        print("  1. Review the analysis and recommendations")
        print("  2. Apply the code fixes to firmware")
        print("  3. Re-compile and re-upload to ESP32")
        print("  4. Verify serial output shows no errors")
        print()

    except Exception as e:
        print()
        print("=" * 80)
        print("ANALYSIS FAILED!")
        print("=" * 80)
        print()
        print(f"Error: {str(e)}")
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
