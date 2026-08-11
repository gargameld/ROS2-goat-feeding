"""Start the static map server and AMCL"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("localization_bringup")
    map_file = PathJoinSubstitution([
        package_share, "maps", "mujoco_arena.yaml"
    ])
    params_file = PathJoinSubstitution([
        package_share, "config", "amcl.yaml"
    ])

    return LaunchDescription([
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {"yaml_filename": map_file, "use_sim_time": True}
            ],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params_file, {"use_sim_time": True}],
            remappings=[("scan", "/front_scan")],
        ),
    ])
