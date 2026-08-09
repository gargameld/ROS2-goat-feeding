"""Launch the simulation interface GUI application."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the simulation interface GUI node."""
    simulation_interface_gui = Node(
        package='simulation_interface_gui',
        executable='simulation_interface',
        name='simulation_interface_gui',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([simulation_interface_gui])
