"""Launch the complete robot control system and simulation GUI."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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

    return LaunchDescription([
        robot_state_publisher,
        robot_control,
        simulation_interface_gui,
        ekf,
        localization,
    ])
