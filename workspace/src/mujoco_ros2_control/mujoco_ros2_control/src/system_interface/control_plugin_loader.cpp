
#include "mujoco_ros2_control/system_interface/control_plugin_loader.hpp"

#include <algorithm>
#include <string>

#include <fmt/compile.h>
#include <fmt/ranges.h>

namespace mujoco_ros2_control
{
namespace
{

// Parameter namespace under which plugins are declared.
constexpr const char* kPluginParameterPrefix = "mujoco_plugins";

/**
 * @brief Collect the unique plugin keys declared under the parameter prefix.
 *
 * A parameter named `mujoco_plugins.my_plugin.type` yields the key `my_plugin`.
 */
std::vector<std::string> discover_plugin_names(const rclcpp::Node::SharedPtr& node,
                                               const std::string& parameter_prefix)
{
  std::vector<std::string> plugin_names;
  const auto list_parameters = node->list_parameters({ parameter_prefix }, 0u);
  const auto init_position = parameter_prefix.size() + 1;
  for (const auto& parameter : list_parameters.names)
  {
    const auto plugin_name =
        parameter.substr(init_position, parameter.find_first_of('.', init_position) - init_position);
    if (std::find(plugin_names.begin(), plugin_names.end(), plugin_name) == plugin_names.end())
    {
      plugin_names.push_back(plugin_name);
    }
  }
  return plugin_names;
}

}  // namespace

void ControlPluginLoader::load(const rclcpp::Node::SharedPtr& node, const mjModel* model, mjData* data,
                               mjSpec* spec, std::recursive_mutex* simulation_mutex,
                               const rclcpp::Logger& logger)
{
  try
  {
    class_loader_ = std::make_unique<pluginlib::ClassLoader<Plugin>>(
        "mujoco_ros2_control_plugins", "mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase");

    const auto plugin_names = discover_plugin_names(node, kPluginParameterPrefix);
    RCLCPP_INFO_EXPRESSION(logger, plugin_names.empty(), "No 'mujoco_plugins' parameter found!");
    RCLCPP_INFO_EXPRESSION(logger, !plugin_names.empty(),
                           "Found 'mujoco_plugins' parameter with the following plugins: %s",
                           fmt::format("{}", fmt::join(plugin_names, ", ")).c_str());

    for (const auto& plugin_name : plugin_names)
    {
      try
      {
        const std::string type_parameter = std::string(kPluginParameterPrefix) + "." + plugin_name + ".type";
        if (!node->has_parameter(type_parameter))
        {
          RCLCPP_WARN(logger, "Plugin parameter '%s' not found, skipping plugin.", type_parameter.c_str());
          continue;
        }

        const std::string plugin_type = node->get_parameter(type_parameter).as_string();
        auto plugin = class_loader_->createSharedInstance(plugin_type);
        plugin->set_simulation_mutex(simulation_mutex);
        plugin->set_mujoco_spec(spec);
        if (plugin->init(node->create_sub_node(plugin_name), model, data))
        {
          plugins_.push_back(plugin);
          RCLCPP_INFO(logger, "Successfully loaded and initialized plugin: %s", plugin_name.c_str());
        }
        else
        {
          RCLCPP_ERROR(logger, "Failed to initialize plugin: %s of type: %s", plugin_name.c_str(),
                       plugin_type.c_str());
          throw std::runtime_error("Failed to initialize plugin: " + plugin_name + " of type: " + plugin_type);
        }
      }
      catch (const pluginlib::PluginlibException& ex)
      {
        RCLCPP_ERROR(logger, "Failed to load plugin '%s': %s", plugin_name.c_str(), ex.what());
        throw;  // re-throw to be caught by the outer catch block
      }
    }
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(logger, "Failed to create plugin loader: %s", ex.what());
  }
}

void ControlPluginLoader::update_all(const mjModel* model, mjData* data)
{
  for (auto& plugin : plugins_)
  {
    plugin->update(model, data);
  }
}

void ControlPluginLoader::cleanup()
{
  for (auto& plugin : plugins_)
  {
    if (plugin)
    {
      plugin->cleanup();
    }
  }
  plugins_.clear();
}

}  // namespace mujoco_ros2_control
