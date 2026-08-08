/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/detail/plugin_helpers.hpp"

#include <algorithm>

namespace mujoco_ros2_control::detail
{

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

}  // namespace mujoco_ros2_control::detail
