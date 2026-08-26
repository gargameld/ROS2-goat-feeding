# MuJoCo ros2_control Simulation

This package contains a ros2_control system interface for the [MuJoCo Simulator](https://mujoco.readthedocs.io/en/3.4.0/overview.html).
It was originally written for simulating robot hardware in NASA Johnson's [iMETRO facility](https://ntrs.nasa.gov/citations/20230015485).

The system interface wraps MuJoCo's [Simulate App](https://github.com/google-deepmind/mujoco/tree/3.4.0/simulate) to provide included functionality.
Because the app is not bundled as a library, we compile it directly from a local install of MuJoCo.

Parts of this library are also based on the MoveIt [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) package.

## URDF Model Conversion

MuJoCo does not support the full feature set of xacro/URDFs in the ROS 2 ecosystem.
Users are required to convert any existing robot description files to an MJCF format.

## Hardware Interface Setup

The MuJoCo hardware interface is shipped as a `ros2_control` plugin. Specify it in your URDF and point to a valid MJCF:

```xml
<ros2_control name="MujocoSystem" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
    <param name="mujoco_model">$(find my_description)/description/scene.xml</param>
  </hardware>
  ...
```

A custom `ros2_control` node is required due to compatibility requirements:

```python
control_node = Node(
    package="mujoco_ros2_control",
    executable="ros2_control_node",
    output="both",
    parameters=[
        {"use_sim_time": True},
        controller_parameters,
    ],
)
```

For the full plugin parameter reference, joint control modes, gripper setup, IMU sensors, and camera configuration, see the [hardware interface documentation](docs/hardware_interface.rst).

## Simulation Topics

The simulator publishes `/clock` and `/mujoco_actuators_states` for interacting with the simulation at runtime.
See the [simulation topics documentation](docs/hardware_interface.rst#simulation-topics) for details.

## Further Documentation

| Document | Description |
|---|---|
| [Hardware Interface](docs/hardware_interface.rst) | Plugin params, joints, sensors, cameras, topics, debugging |
| [Modeling Tips](docs/modeling_tips.rst) | Tips for modeling complex geometries in MuJoCo |
| [Developers Guide](../doc/development.rst) | Development workflows (Docker, pixi) |
