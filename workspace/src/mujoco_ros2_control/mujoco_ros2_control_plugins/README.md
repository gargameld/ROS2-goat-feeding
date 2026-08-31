# mujoco_ros2_control_plugins

This package provides a plugin interface for extending the functionality of `mujoco_ros2_control`.

## Documentation

Full documentation is maintained in RST format:

- **[MuJoCo ROS 2 Control Plugins](doc/plugins.rst)** — available plugins, usage, configuration, and instructions for writing your own plugin.

## Available Plugins

| Plugin | Description |
|---|---|
| `Mujoco3dLidarPlugin` | Publishes `mujoco.plugin.lidar` sensors as `LaserScan` (single row) or `PointCloud2` (multi row) |
| `StateCapturePlugin` | Buffers simulation time and `qpos` and flushes them to a CSV file |
| `ObstacleControlPlugin` | Pins the free obstacle body to a configurable, service-controlled position |
| `FoodControlPlugin` | Throws food items into configured parking areas |
| `SimulationStateProviderPlugin` | Returns live generalized positions and obstacle state |

## Quick Start

Load plugins by passing a parameters file to the `mujoco_ros2_control` node:

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      state_capture:
        type: "mujoco_ros2_control_plugins/StateCapturePlugin"
```

For full usage details, service/topic interfaces, parameters, and a guide to writing your own
plugin, see [`doc/plugins.rst`](doc/plugins.rst).

## See Also

- Main package: [mujoco_ros2_control](../mujoco_ros2_control/)
