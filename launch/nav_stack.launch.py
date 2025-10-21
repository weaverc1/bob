"""
Navigation stack launch file template
Launches Nav2 and SLAM components
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Launch SLAM or use pre-made map'
        ),

        # TODO: Include Nav2 bringup
        # TODO: Include SLAM Toolbox or Cartographer
        # TODO: Add parameter files for planners and controllers
        # TODO: Add map server (if not using SLAM)
    ])
