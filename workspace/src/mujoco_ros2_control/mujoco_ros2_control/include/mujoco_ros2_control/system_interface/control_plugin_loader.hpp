
#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <mujoco/mujoco.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control
{

/**
 * @brief Loads and drives the ros2_control plugins configured for the simulation.
 *
 * These are `mujoco_ros2_control_plugins` loaded through pluginlib. They are unrelated to the
 * MuJoCo engine extensions registered by load_mujoco_extensions().
 *
 * Plugins are declared under the `mujoco_plugins` parameter namespace. Each unique key below it
 * names one plugin and must carry a `type` parameter holding its pluginlib class name:
 *
 *     mujoco_plugins:
 *       my_plugin:
 *         type: "my_package/MyPlugin"
 *
 * Each plugin is initialized with a sub-node named after its key, so its own parameters resolve
 * under that namespace.
 */
class ControlPluginLoader
{
public:
  using Plugin = mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase;

  /**
   * @brief Instantiate and initialize every configured plugin.
   *
   * Failures to load or initialize an individual plugin are logged; the simulation continues
   * with whichever plugins did come up.
   *
   * @param node Node whose parameters declare the plugins, and whose sub-nodes they are given.
   * @param model MuJoCo model handed to each plugin.
   * @param data MuJoCo data handed to each plugin.
   * @param spec Editable specification behind the model; may be null for binary MJB models.
   * @param simulation_mutex Mutex guarding the live model and data.
   */
  void load(const rclcpp::Node::SharedPtr& node, const mjModel* model, mjData* data, mjSpec* spec,
            std::recursive_mutex* simulation_mutex, const rclcpp::Logger& logger);

  /**
   * @brief Run every loaded plugin's update.
   * @note Called from the real-time read() cycle.
   */
  void update_all(const mjModel* model, mjData* data);

  /**
   * @brief Tear every loaded plugin down. Safe to call when nothing was loaded.
   */
  void cleanup();

private:
  std::unique_ptr<pluginlib::ClassLoader<Plugin>> class_loader_;
  std::vector<std::shared_ptr<Plugin>> plugins_;
};

}  // namespace mujoco_ros2_control
