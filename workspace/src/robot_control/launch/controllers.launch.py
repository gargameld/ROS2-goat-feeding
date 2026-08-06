from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_spawner(controller_name, controller_manager):
    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_{controller_name}",
        output="screen",
        arguments=[
            controller_name,
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            "120",
            "--switch-timeout",
            "120",
        ],
    )


def generate_launch_description():
    controller_manager = LaunchConfiguration("controller_manager")

    declare_controller_manager = DeclareLaunchArgument(
        "controller_manager",
        default_value="/controller_manager",
        description="Name of the controller manager node",
    )

    arm_spawner = make_spawner(
        "arm_trajectory_controller",
        controller_manager,
    )

    remaining_controller_names = (
        "joint_state_broadcaster",
        "mecanum_drive_controller",
        "gripper_controller",
        "imu_sensor_broadcaster",
    )

    remaining_spawners = [
        make_spawner(controller_name, controller_manager)
        for controller_name in remaining_controller_names
    ]

    def start_remaining_controllers(event, _context):
        if event.returncode != 0:
            return [LogInfo(
                msg="ERROR: Arm controller failed to activate; remaining controllers will not be spawned."
            )]
        return remaining_spawners

    start_remaining_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_spawner,
            on_exit=start_remaining_controllers,
        )
    )

    return LaunchDescription([
        declare_controller_manager,
        start_remaining_after_arm,
        arm_spawner,
    ])
