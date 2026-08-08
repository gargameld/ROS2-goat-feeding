from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    controller_names = (
        "arm_trajectory_controller",
        "joint_state_broadcaster",
        "mecanum_drive_controller",
        "gripper_controller",
        "imu_sensor_broadcaster",
    )

    controller_spawners = [
        make_spawner(controller_name, controller_manager)
        for controller_name in controller_names
    ]

    return LaunchDescription([
        declare_controller_manager,
        *controller_spawners,
    ])
