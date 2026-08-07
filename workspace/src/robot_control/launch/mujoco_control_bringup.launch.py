from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    mujoco_model_path = LaunchConfiguration("mujoco_model_path")
    mujoco_start_delay = LaunchConfiguration("mujoco_start_delay")

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

    declare_mujoco_start_delay = DeclareLaunchArgument(
        "mujoco_start_delay",
        default_value="3.0",
        description="Seconds to wait before starting mujoco_ros2_control",
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

    controllers_file = PathJoinSubstitution([
        FindPackageShare("robot_control"),
        "config",
        "controllers.yaml",
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
        parameters=[
            robot_description,
            ParameterFile(mujoco_plugins_file),
            ParameterFile(controllers_file),
        ],
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_control"),
                "launch",
                "controllers.launch.py",
            ])
        ),
        launch_arguments={
            "controller_manager": "/controller_manager",
        }.items(),
    )

    simulation_interface_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("simulation_interface_gui"),
                "launch",
                "simulation_interface_gui.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    delayed_mujoco_ros2_control_node = TimerAction(
        period=mujoco_start_delay,
        actions=[mujoco_ros2_control_node],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_mujoco_model_path,
        declare_mujoco_start_delay,
        robot_state_publisher_node,
        spawn_controllers,
        delayed_mujoco_ros2_control_node,
        simulation_interface_gui,
    ])
