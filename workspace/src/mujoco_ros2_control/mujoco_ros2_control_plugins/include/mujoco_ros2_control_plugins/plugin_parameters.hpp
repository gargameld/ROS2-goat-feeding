#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_PARAMETERS_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_PARAMETERS_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control_plugins
{

/** Generic lookup for parameters stored below mujoco_plugins.<plugin_name>. */
class PluginParameters
{
public:
  explicit PluginParameters(rclcpp::Node::SharedPtr node);

  bool get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                     std::string& value) const;
  bool get_parameter(const std::string& plugin_name, const std::string& parameter_name, double& value) const;
  bool get_parameter(const std::string& plugin_name, const std::string& parameter_name, int64_t& value) const;
  bool get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                     std::vector<double>& value) const;

  bool get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                     const std::string& default_value, std::string& value) const;
  bool get_parameter(const std::string& plugin_name, const std::string& parameter_name, double default_value,
                     double& value) const;
  bool get_parameter(const std::string& plugin_name, const std::string& parameter_name, int64_t default_value,
                     int64_t& value) const;
  bool get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                     const std::vector<double>& default_value, std::vector<double>& value) const;

private:
  std::string full_name(const std::string& plugin_name, const std::string& parameter_name) const;

  rclcpp::Node::SharedPtr node_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_PARAMETERS_HPP_
