
#pragma once

namespace mujoco_ros2_control
{

/**
 * @brief Register every MuJoCo engine extension available in this installation.
 *
 * These are MuJoCo's own native plugins (shared libraries implementing mjpPlugin, such as the
 * lidar sensor), not the ros2_control plugins loaded by ControlPluginLoader. They must be
 * registered before a model is compiled, because the MJCF resolves its `<extension>` declarations
 * against the registered set.
 *
 * Libraries are discovered in two places: the `mujoco_plugin` directory beside the running
 * executable, and every ROS package that registers a `mujoco_plugins` ament resource.
 */
void load_mujoco_extensions();

}  // namespace mujoco_ros2_control
