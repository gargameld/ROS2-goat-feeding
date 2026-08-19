from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("full_robot", package_name="moveit_config").to_moveit_configs()
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        generate_move_group_launch(moveit_config),
    ])
