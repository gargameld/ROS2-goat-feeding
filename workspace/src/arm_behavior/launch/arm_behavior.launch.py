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
        .planning_pipelines(
            default_planning_pipeline='ompl', pipelines=['ompl'], load_all=False)
        .to_moveit_configs()
    )
    behavior_config = os.path.join(
        get_package_share_directory('arm_behavior'),
        'config',
        'arm_behavior.yaml',
    )
    default_environment_config = os.path.join(
        get_package_share_directory('moveit_config'),
        'config',
        'environment_boxes.yaml',
    )
    environment_config = LaunchConfiguration('environment_config')

    arm_behavior = Node(
        package='arm_behavior',
        executable='arm_behavior_node',
        name='arm_behavior',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            behavior_config,
            {'use_sim_time': use_sim_time},
        ],
        # Runs in the root namespace so its TF listener reads the global
        # /tf and /tf_static (where AMCL/EKF/robot_state_publisher publish),
        # matching move_group. The remaps below are now identity but kept
        # explicit to document the topics the node depends on.
        remappings=[
            ('joint_states', '/joint_states'),
            ('get_planning_scene', '/get_planning_scene'),
            ('apply_planning_scene', '/apply_planning_scene'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    )

    environment_loader = Node(
        package='arm_behavior',
        executable='environment_loader',
        name='environment_loader',
        output='screen',
        parameters=[environment_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'environment_config',
            default_value=default_environment_config,
            description='YAML file containing map-frame MoveIt collision boxes',
        ),
        arm_behavior,
        environment_loader,
    ])
