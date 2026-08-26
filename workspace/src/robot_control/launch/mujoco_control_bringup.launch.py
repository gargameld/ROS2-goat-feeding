from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mujoco_start_delay = LaunchConfiguration("mujoco_start_delay")
    mujoco_model_path = "mujoco_model/scene.xml"

    declare_mujoco_start_delay = DeclareLaunchArgument(
        "mujoco_start_delay",
        default_value="3.0",
        description="Seconds to wait before starting mujoco_ros2_control",
    )
    controllers_file = (
        Path(get_package_share_directory("robot_control"))
        / "config"
        / "controllers.yaml"
    )

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("robot_description"),
                "urdf",
                "full_robot.xacro",
            ]),
        ]),
        value_type=str,
    )

    robot_description = {
        "robot_description": robot_description_content,
        "use_sim_time": True,
        "mujoco_model_path": mujoco_model_path,
    }

    mujoco_plugins_file = PathJoinSubstitution([
        FindPackageShare("mujoco_ros2_control_plugins"),
        "config",
        "mujoco_ros2_control_plugins.yaml",
    ])

    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            robot_description,
            ParameterFile(mujoco_plugins_file),
            ParameterFile(str(controllers_file)),
        ],
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_control"),
                "launch",
                "controllers.launch.py",
            ])
        ),
        launch_arguments={
            "controller_manager": "/controller_manager",
        }.items(),
    )

    spawn_controllers_after_manager_starts = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco_ros2_control_node,
            on_start=[spawn_controllers],
        )
    )

    delayed_mujoco_ros2_control_node = TimerAction(
        period=mujoco_start_delay,
        actions=[mujoco_ros2_control_node],
    )

    return LaunchDescription([
        declare_mujoco_start_delay,
        spawn_controllers_after_manager_starts,
        delayed_mujoco_ros2_control_node,
    ])
