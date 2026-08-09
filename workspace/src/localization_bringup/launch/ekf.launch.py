"""Fuse wheel odometry and IMU data into a filtered odometry estimate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_share = FindPackageShare("localization_bringup")
    params_file = PathJoinSubstitution([
        package_share, "config", "ekf.yaml"
    ])
    odom_topic = "/mecanum_drive_controller/odometry"
    imu_topic = "/imu_sensor_broadcaster/imu"
    output_topic = "/odometry/filtered"

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[params_file, {"use_sim_time": True}],
            remappings=[
                ("odom", odom_topic),
                ("imu", imu_topic),
                ("odometry/filtered", output_topic),
            ],
        ),
    ])
