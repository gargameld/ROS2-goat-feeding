"""Launch the MoveIt-backed arm behavior node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    moveit_config = (
        MoveItConfigsBuilder('full_robot', package_name='moveit_config')
        .to_moveit_configs()
    )
    behavior_config = os.path.join(
        get_package_share_directory('arm_behavior'),
        'config',
        'arm_behavior.yaml',
    )

    arm_behavior = Node(
        package='arm_behavior',
        executable='arm_behavior_node',
        namespace='arm',
        name='arm_behavior',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            behavior_config,
            {'use_sim_time': use_sim_time},
        ],
        # PlanningSceneInterface creates clients relative to the node namespace,
        # while move_group intentionally runs in the root namespace.
        remappings=[
            ('get_planning_scene', '/get_planning_scene'),
            ('apply_planning_scene', '/apply_planning_scene'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        arm_behavior,
    ])
