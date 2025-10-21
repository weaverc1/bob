"""
Launch file template for spawning the testbot in Gazebo
Smaller indoor test platform
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),

        # TODO: Similar structure to mower_gazebo.launch.py but with testbot URDF
        # TODO: Use indoor_arena_template.world by default
    ])
