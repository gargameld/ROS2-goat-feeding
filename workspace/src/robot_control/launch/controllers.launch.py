from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event import Event
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class ControllerSpawnersComplete(Event):
    """Signal to the top-level launch that controller startup is complete."""

    name = "robot_control.ControllerSpawnersComplete"


def make_spawner(controller_name, controller_manager, params_file):
    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_{controller_name}",
        output="screen",
        arguments=[
            controller_name,
            "--controller-manager",
            controller_manager,
            "--param-file",
            params_file,
            "--controller-manager-timeout",
            "120",
            "--switch-timeout",
            "120",
        ],
    )


def generate_launch_description():
    controller_manager = LaunchConfiguration("controller_manager")
    params_file = PathJoinSubstitution([
        FindPackageShare("robot_control"),
        "config",
        "controllers.yaml",
    ])

    declare_controller_manager = DeclareLaunchArgument(
        "controller_manager",
        default_value="/controller_manager",
        description="Name of the controller manager node",
    )

    controller_names = (
        "joint_state_broadcaster",
        "imu_sensor_broadcaster",
        "mecanum_drive_controller",
        "arm_trajectory_controller",
        "gripper_controller",
    )

    controller_spawners = [
        make_spawner(controller_name, controller_manager, params_file)
        for controller_name in controller_names
    ]

    # A controller must be active before controllers that depend on its state
    # interfaces are spawned.  Chain the spawners rather than launching them
    # concurrently.  The final spawner emits an event consumed by the
    # top-level system launch.
    sequential_spawners = [controller_spawners[0]]
    for previous_spawner, next_spawner in zip(
        controller_spawners, controller_spawners[1:]
    ):
        sequential_spawners.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=previous_spawner,
                    on_exit=[next_spawner],
                )
            )
        )

    emit_controller_spawners_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawners[-1],
            on_exit=[
                LogInfo(msg="Final controller spawner exited."),
                EmitEvent(event=ControllerSpawnersComplete()),
            ],
        )
    )

    return LaunchDescription([
        declare_controller_manager,
        *sequential_spawners,
        emit_controller_spawners_complete,
    ])
