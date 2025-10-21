#!/usr/bin/env python3
"""
Main entry point for the AI Mower Crew
"""

import sys
from pathlib import Path
from crew import AiMowerCrew
import yaml


def load_hardware_config(config_path: str = "config/hardware_inventory.yaml") -> dict:
    """Load hardware inventory configuration"""
    config_file = Path(__file__).parent / config_path

    if not config_file.exists():
        print(f"Warning: Hardware config file not found at {config_file}")
        print("Using default configuration...")
        return {}

    with open(config_file, 'r') as f:
        return yaml.safe_load(f)


def run():
    """
    Run the AI Mower Crew
    """
    print("=" * 80)
    print("AI MOWER CREW - Autonomous Lawn Mower Development System")
    print("=" * 80)
    print()
    print("Initializing crew with 10 specialized agents:")
    print("  1. System Architect")
    print("  2. Safety Engineer")
    print("  3. Navigation Specialist")
    print("  4. Differential Drive Specialist")
    print("  5. ROS Code Hunter")
    print("  6. Simulator")
    print("  7. Test Specialist")
    print("  8. The Realist")
    print("  9. ROS Infrastructure Builder")
    print("  10. Controller & Plugin Integrator")
    print()
    print("-" * 80)

    # Load hardware configuration
    hardware_config = load_hardware_config()

    # Initialize the crew
    crew_instance = AiMowerCrew()

    # Prepare inputs for the crew
    inputs = {
        'ros_distro': 'jazzy',
        'compute_platform': 'Raspberry Pi 5',
        'hardware_config': hardware_config,
        'project_root': str(Path(__file__).parent),
    }

    print("\nCrew Configuration:")
    print(f"  ROS Distribution: {inputs['ros_distro']}")
    print(f"  Compute Platform: {inputs['compute_platform']}")
    print(f"  Hardware Config Loaded: {'Yes' if hardware_config else 'No (using defaults)'}")
    print()
    print("-" * 80)
    print("\nStarting crew execution...")
    print("=" * 80)
    print()

    # Run the crew
    try:
        result = crew_instance.crew().kickoff(inputs=inputs)

        print()
        print("=" * 80)
        print("CREW EXECUTION COMPLETED")
        print("=" * 80)
        print()
        print("Results:")
        print(result)

        # Save results to file
        output_dir = Path(__file__).parent / "output"
        output_dir.mkdir(exist_ok=True)

        output_file = output_dir / "crew_results.md"
        with open(output_file, 'w') as f:
            f.write("# AI Mower Crew Execution Results\n\n")
            f.write(str(result))

        print()
        print(f"Results saved to: {output_file}")

    except Exception as e:
        print()
        print("=" * 80)
        print("ERROR DURING CREW EXECUTION")
        print("=" * 80)
        print(f"\n{type(e).__name__}: {e}")
        print()
        import traceback
        traceback.print_exc()
        sys.exit(1)


def train():
    """
    Train the crew for future executions
    """
    print("Training the AI Mower Crew...")
    crew_instance = AiMowerCrew()

    inputs = {
        'ros_distro': 'jazzy',
        'compute_platform': 'Raspberry Pi 5',
    }

    try:
        crew_instance.crew().train(
            n_iterations=int(sys.argv[1]) if len(sys.argv) > 1 else 1,
            inputs=inputs
        )
    except Exception as e:
        print(f"Training error: {e}")
        raise


def replay():
    """
    Replay the crew execution from a specific task
    """
    print("Replaying AI Mower Crew execution...")
    crew_instance = AiMowerCrew()

    try:
        crew_instance.crew().replay(task_id=sys.argv[1] if len(sys.argv) > 1 else None)
    except Exception as e:
        print(f"Replay error: {e}")
        raise


def test():
    """
    Test the crew with a subset of tasks
    """
    print("Testing AI Mower Crew...")
    crew_instance = AiMowerCrew()

    inputs = {
        'ros_distro': 'jazzy',
        'compute_platform': 'Raspberry Pi 5',
    }

    try:
        crew_instance.crew().test(
            n_iterations=int(sys.argv[1]) if len(sys.argv) > 1 else 1,
            inputs=inputs
        )
    except Exception as e:
        print(f"Test error: {e}")
        raise


if __name__ == "__main__":
    # Check for command line arguments
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()

        if command == "train":
            train()
        elif command == "replay":
            replay()
        elif command == "test":
            test()
        elif command == "help":
            print("AI Mower Crew - Usage:")
            print("  python main.py          - Run the crew normally")
            print("  python main.py train    - Train the crew")
            print("  python main.py replay   - Replay crew execution")
            print("  python main.py test     - Test the crew")
            print("  python main.py help     - Show this help message")
        else:
            print(f"Unknown command: {command}")
            print("Run 'python main.py help' for usage information")
            sys.exit(1)
    else:
        run()
