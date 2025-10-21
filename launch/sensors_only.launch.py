"""
Sensors-only launch file
For testing and validating sensor functionality incrementally
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # TODO: Add Lidar driver launch
        # TODO: Add IMU driver launch
        # TODO: Add camera driver launch
        # TODO: Add GPS driver launch (if applicable)
        # TODO: Add static transforms for sensor frames
    ])
