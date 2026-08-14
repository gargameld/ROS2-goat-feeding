"""Launch the complete robot control system and simulation GUI."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handler import EventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'robot_state_publisher.launch.py',
            ])))

    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_control'),
                'launch',
                'mujoco_control_bringup.launch.py',
            ])
        )
    )

    simulation_interface_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('simulation_interface_gui'),
                'launch',
                'simulation_interface_gui.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization_bringup'),
                'launch',
                'ekf.launch.py',
            ])
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization_bringup'),
                'launch',
                'localization.launch.py',
            ])
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('navigation_bringup'),
                'launch',
                'navigation.launch.py',
            ])
        )
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'velocity_smoother',
                'collision_monitor',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )

    start_lifecycle_manager_after_controllers = RegisterEventHandler(
        EventHandler(
            matcher=lambda event: getattr(event, 'name', None) ==
            'robot_control.ControllerSpawnersComplete',
            entities=[lifecycle_manager],
            handle_once=True,
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        robot_control,
        simulation_interface_gui,
        ekf,
        localization,
        navigation,
        start_lifecycle_manager_after_controllers,
    ])
