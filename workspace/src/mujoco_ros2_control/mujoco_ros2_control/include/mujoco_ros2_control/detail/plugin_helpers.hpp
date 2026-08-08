/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <string>
#include <vector>

#include <rclcpp/node.hpp>

namespace mujoco_ros2_control::detail
{

std::vector<std::string> discover_plugin_names(const rclcpp::Node::SharedPtr& node,
                                               const std::string& parameter_prefix);

}  // namespace mujoco_ros2_control::detail
