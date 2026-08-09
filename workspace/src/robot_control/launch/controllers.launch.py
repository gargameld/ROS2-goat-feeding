from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    return LaunchDescription([
        declare_controller_manager,
        *controller_spawners,
    ])
