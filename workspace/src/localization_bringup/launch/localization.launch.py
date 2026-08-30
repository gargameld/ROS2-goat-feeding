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
    keepout_mask_file = PathJoinSubstitution([
        package_share, "maps", "mujoco_keepout.yaml"
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
            package="nav2_map_server",
            executable="map_server",
            name="keepout_filter_mask_server",
            output="screen",
            parameters=[{
                "yaml_filename": keepout_mask_file,
                "topic_name": "/keepout_filter_mask",
                "frame_id": "map",
                "use_sim_time": True,
            }],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="keepout_costmap_filter_info_server",
            output="screen",
            parameters=[{
                "type": 0,
                "filter_info_topic": "/keepout_costmap_filter_info",
                "mask_topic": "/keepout_filter_mask",
                "base": 0.0,
                "multiplier": 1.0,
                "use_sim_time": True,
            }],
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
