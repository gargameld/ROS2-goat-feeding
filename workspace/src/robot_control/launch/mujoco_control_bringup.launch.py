from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    mujoco_model_path = LaunchConfiguration("mujoco_model_path")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    declare_mujoco_model_path = DeclareLaunchArgument(
        "mujoco_model_path",
        default_value="src/robot_description/mjcf/scene.xml",
        description="Absolute path to the MuJoCo MJCF model file",
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
        "use_sim_time": use_sim_time,
        "mujoco_model_path": mujoco_model_path,
    }

    mujoco_plugins_file = PathJoinSubstitution([
        FindPackageShare("mujoco_ros2_control_plugins"),
        "config",
        "mujoco_ros2_control_plugins.yaml",
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[robot_description, ParameterFile(mujoco_plugins_file)],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_mujoco_model_path,
        mujoco_ros2_control_node,
        robot_state_publisher_node
    ])
