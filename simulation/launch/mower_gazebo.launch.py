"""
Launch file template for spawning the mower in Gazebo
To be populated by ROS Infrastructure Builder and Controller & Plugin Integrator agents
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='backyard_template.world')

    # TODO: Add actual paths once package structure is established
    # urdf_file = PathJoinSubstitution([FindPackageShare('ai_mower_description'), 'urdf', 'mower_template.urdf.xacro'])
    # world_path = PathJoinSubstitution([FindPackageShare('ai_mower_simulation'), 'worlds', world_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),

        DeclareLaunchArgument(
            'world',
            default_value='backyard_template.world',
            description='Gazebo world file to load'
        ),

        # TODO: Add Gazebo server launch
        # TODO: Add Gazebo client launch
        # TODO: Add robot state publisher
        # TODO: Add spawn entity node
        # TODO: Add controller manager and spawners
    ])
