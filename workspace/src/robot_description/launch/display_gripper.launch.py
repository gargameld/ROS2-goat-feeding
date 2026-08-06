from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mujoco_model_path = LaunchConfiguration("mujoco_model_path")

    declare_mujoco_model_path = DeclareLaunchArgument(
        "mujoco_model_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("robot_description"),
            "mjcf",
            "scene.xml"
        ]),
        description="Path to MuJoCo MJCF model file",
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                PathJoinSubstitution([
                    FindPackageShare("robot_description"),
                    "urdf",
                    "full_robot.xacro"
                ])
            ]),
            value_type=str
        )
    }

    rviz_config = PathJoinSubstitution([
        FindPackageShare("robot_description"),
        "config",
        "rviz_config.rviz"
    ])

    mujoco_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_control"),
                "launch",
                "mujoco_control_bringup.launch.py"
            ])
        ),
        launch_arguments={
            "mujoco_model_path": mujoco_model_path,
        }.items()
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[robot_description],
        output="screen",
    )

    gripper_debugger_node = Node(
        package="robot_description",
        executable="gripper_debugger",
        name="gripper_debugger",
        output="screen",
    )

    rviz_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        declare_mujoco_model_path,
        mujoco_control_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gripper_debugger_node,
        rviz_node,
    ])