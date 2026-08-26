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

The simulator publishes `/clock`. Joint states reach ROS through the normal `ros2_control` state
interfaces, so use `joint_state_broadcaster` for `/joint_states`.
See the [simulation topics documentation](docs/hardware_interface.rst#simulation-topics) for details.

The simulation always runs headless. There is no viewer window, render loop or keyboard stepping.

## Source Layout

The package is organised by responsibility. Each folder answers one question:

| Folder | Question it answers |
|---|---|
| `system_interface/` | How does the URDF/`ros2_control` model map onto MuJoCo? |
| `simulation/` | How does MuJoCo actually advance? |
| `sensors/` | How do MuJoCo cameras become ROS data? |

`mujoco_system_interface.hpp` is the public entry point `pluginlib` loads; `data.hpp` holds the
plain data types shared across all three areas.

### `system_interface/`

| File | Responsibility |
|---|---|
| `simulation_configuration` | Read the `<hardware>` params (model path, speed factor, camera rate) and the node options |
| `mujoco_model_validation` | Reject a compiled MJCF that ros2_control cannot address (unnamed joints) |
| `mujoco_actuator_discovery` | Classify MuJoCo actuators, resolve which joint each drives, register unactuated joints as passive |
| `joint_command_setup` | Register the URDF joints, match each one's command interfaces to its actuator and seed initial values |
| `imu_sensor_setup` | Resolve one ros2_control IMU to its `_quat` / `_gyro` / `_accel` MJCF sensors |
| `sensor_registration` | Walk the hardware info's sensors and build the data container each `mujoco_type` needs |
| `interface_export` | Build the state and command interface vectors ros2_control exports |
| `initial_state` | Apply the starting pose to MuJoCo and sync passive joints |
| `command_mode_switching` | Enable/disable position, velocity or effort control on a mode switch |
| `state_reading` | Copy MuJoCo actuator and IMU state into the exported interfaces each cycle |
| `joint_actuator_mapping` | Carry states from actuators to joints and commands from joints to actuators |
| `control_plugin_loader` | Load and drive the `mujoco_ros2_control_plugins` declared under `mujoco_plugins` |

### `simulation/`

| File | Responsibility |
|---|---|
| `mujoco_simulation` | Lifecycle only: construct the Simulate app, own the model/data, start and stop the physics thread |
| `physics_loop` | The loop itself and every step it performs: control inputs, plugin forces, `mj_step`, divergence handling |
| `mujoco_model_loader` | Compile an MJCF (or load an MJB) and report errors, warnings and load time |
| `mujoco_extension_loader` | Register MuJoCo's own engine extensions (e.g. the lidar sensor) before the model compiles |
| `headless_adapter` | The no-op UI adapter the Simulate app is constructed with |
| `mujoco_simulation_clock` | Simulation time end to end: read it, wait on it, and publish it to `/clock` |
| `physics_loop_synchronizer` | Hold physics back until controllers have written, and serve pause/resume |

### Two kinds of "plugin"

The package deals with two unrelated plugin systems, kept in separate files so they are not
confused:

- **MuJoCo engine extensions** — native MuJoCo plugins (shared libraries implementing `mjpPlugin`,
  such as `mujoco_3d_lidar`). Registered by `simulation/mujoco_extension_loader` *before* the model
  is compiled, because the MJCF resolves `<extension>` declarations against the registered set.
- **ros2_control plugins** — `mujoco_ros2_control_plugins` loaded through pluginlib and updated
  every control cycle. Handled by `system_interface/control_plugin_loader`.

## Further Documentation

| Document | Description |
|---|---|
| [Hardware Interface](docs/hardware_interface.rst) | Plugin params, joints, sensors, cameras, topics |
| [Modeling Tips](docs/modeling_tips.rst) | Tips for modeling complex geometries in MuJoCo |
