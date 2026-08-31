"""Launch the grasp pose provider node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_provider_config = os.path.join(
        get_package_share_directory('grasp_pose_provider'),
        'config',
        'grasp_pose_provider.yaml',
    )
    provider_config = LaunchConfiguration('provider_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'provider_config',
            default_value=default_provider_config,
            description='YAML file holding the grasp pose provider parameters',
        ),
        Node(
            package='grasp_pose_provider',
            executable='grasp_pose_provider_node',
            name='grasp_pose_provider',
            output='screen',
            parameters=[provider_config],
        ),
    ])
