"""
Differential drive validation test
Tests odometry, wheel control, and kinematics
"""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        # TODO: Launch diff drive controller
        # TODO: Launch odometry publisher
        # TODO: Launch test scripts that:
        #   - Command specific velocities
        #   - Validate odometry output
        #   - Test wheel encoder readings
        #   - Validate kinematic model
    ])
