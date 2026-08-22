"""Launch the grasp pose provider node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='grasp_pose_provider',
            executable='grasp_pose_provider_node',
            name='grasp_pose_provider',
            output='screen',
        ),
    ])
