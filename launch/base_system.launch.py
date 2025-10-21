"""
Base system launch file template
Launches core ROS2 nodes for the mower (without navigation)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when true'
        ),

        # TODO: Add robot state publisher
        # TODO: Add joint state publisher (if needed)
        # TODO: Add controller manager
        # TODO: Add sensor drivers
        # TODO: Add odometry fusion node
    ])
