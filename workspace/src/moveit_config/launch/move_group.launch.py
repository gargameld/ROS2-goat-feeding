from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("full_robot", package_name="moveit_config").to_moveit_configs()

    # Mirrors moveit_configs_utils.launches.generate_move_group_launch, declared
    # explicitly so move_group's log level can be raised when planning needs
    # debugging (append --ros-args --log-level debug to `arguments` below).
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": ":0"},
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        move_group,
    ])
