"""Launch the Nav2 controller, planner, and NavigateToPose navigator."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('navigation_bringup')

    controller_params = PathJoinSubstitution(
        [package_share, 'config', 'controller_server.yaml']
    )
    planner_params = PathJoinSubstitution(
        [package_share, 'config', 'planner_server.yaml']
    )
    bt_navigator_params = PathJoinSubstitution(
        [package_share, 'config', 'bt_navigator.yaml']
    )
    navigate_to_pose_tree = PathJoinSubstitution(
        [package_share, 'behavior_trees', 'navigate_to_pose.xml']
    )

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_params, {'use_sim_time': True}],
            remappings=[
                ('cmd_vel', '/mecanum_drive_controller/reference'),
            ],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_params, {'use_sim_time': True}],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_navigator_params,
                {
                    'use_sim_time': True,
                    'default_nav_to_pose_bt_xml': navigate_to_pose_tree,
                },
            ],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[bt_navigator_params, {'use_sim_time': True}],
        ),
    ])
