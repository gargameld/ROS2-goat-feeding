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

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        )
    )

    arm_behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_behavior'),
                'launch',
                'arm_behavior.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    grasp_pose_provider = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('grasp_pose_provider'),
                'launch',
                'grasp_pose_provider.launch.py',
            ])
        )
    )

    gpd_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gpd_ros2'),
                'launch',
                'detect_constrained_grasps.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    grasp_rvis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('grasp_rvis'),
                'launch',
                'grasp_rvis.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
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
        move_group,
        arm_behavior,
        grasp_pose_provider,
        gpd_server,
        grasp_rvis,
        start_lifecycle_manager_after_controllers,
    ])
