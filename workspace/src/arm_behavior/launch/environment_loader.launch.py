"""Load the static MuJoCo environment into the MoveIt planning scene."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_environment_config = os.path.join(
        get_package_share_directory('moveit_config'),
        'config',
        'environment_boxes.yaml',
    )
    environment_config = LaunchConfiguration('environment_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'environment_config',
            default_value=default_environment_config,
            description='YAML file containing map-frame MoveIt collision boxes',
        ),
        Node(
            package='arm_behavior',
            executable='environment_loader',
            name='environment_loader',
            output='screen',
            parameters=[environment_config],
        ),
    ])
