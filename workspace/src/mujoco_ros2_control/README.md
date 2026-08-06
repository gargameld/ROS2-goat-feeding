# MuJoCo ros2_control

This directory contains the runtime packages needed to run ROS 2 controllers
against the MuJoCo physics simulator in this workspace.

## Contents

- `mujoco_ros2_control` - core system interface and conversion utilities
- `mujoco_ros2_control_msgs` - message and service definitions used by the core
- `mujoco_ros2_control_plugins` - simulation plugins used by this workspace

The demo packages, test packages, Docker configuration, CI configuration, and
alternative Pixi development environment were removed because they are not
needed at runtime.

## Build

Install dependencies with `rosdep`, then build from the ROS workspace root:

```bash
colcon build --symlink-install --packages-select \
  mujoco_ros2_control_msgs \
  mujoco_ros2_control_plugins \
  mujoco_ros2_control \
  robot_control
```

See [mujoco_ros2_control/README.md](./mujoco_ros2_control/README.md) for the
hardware interface and model configuration documentation.

## License

The retained upstream packages are distributed under the Apache-2.0 license.
See [LICENSE](./LICENSE).
