"""Launch the simulation interface GUI application."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the simulation interface GUI node."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the simulation clock for stamped velocity commands.',
    )
    simulation_interface_gui = Node(
        package='simulation_interface_gui',
        executable='simulation_interface',
        name='simulation_interface_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([declare_use_sim_time, simulation_interface_gui])
