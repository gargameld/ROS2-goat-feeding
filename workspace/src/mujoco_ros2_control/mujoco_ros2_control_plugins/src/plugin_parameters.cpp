// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mujoco_ros2_control_plugins/plugin_parameters.hpp"

#include <exception>
#include <utility>

namespace mujoco_ros2_control_plugins
{

namespace
{

template <typename Value>
bool read_required(const rclcpp::Node::SharedPtr& node, const std::string& name, Value& value)
{
  if (!node->has_parameter(name))
  {
    RCLCPP_ERROR(node->get_logger(), "Required parameter '%s' is missing.", name.c_str());
    return false;
  }

  try
  {
    value = node->get_parameter(name).get_value<Value>();
    return true;
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(node->get_logger(), "Could not read parameter '%s': %s", name.c_str(), exception.what());
    return false;
  }
}

template <typename Value>
bool read_or_declare(const rclcpp::Node::SharedPtr& node, const std::string& name, const Value& default_value,
                     Value& value)
{
  try
  {
    if (!node->has_parameter(name))
    {
      node->declare_parameter(name, default_value);
    }
    value = node->get_parameter(name).get_value<Value>();
    return true;
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(node->get_logger(), "Could not declare or read parameter '%s': %s", name.c_str(), exception.what());
    return false;
  }
}

}  // namespace

PluginParameters::PluginParameters(rclcpp::Node::SharedPtr node) : node_(std::move(node))
{
}

bool PluginParameters::get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                                     std::string& value) const
{
  return read_required(node_, full_name(plugin_name, parameter_name), value);
}

bool PluginParameters::get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                                     double& value) const
{
  return read_required(node_, full_name(plugin_name, parameter_name), value);
}

bool PluginParameters::get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                                     int64_t& value) const
{
  return read_required(node_, full_name(plugin_name, parameter_name), value);
}

bool PluginParameters::get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                                     std::vector<double>& value) const
{
  return read_required(node_, full_name(plugin_name, parameter_name), value);
}

bool PluginParameters::get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                                     const std::string& default_value, std::string& value) const
{
  return read_or_declare(node_, full_name(plugin_name, parameter_name), default_value, value);
}

bool PluginParameters::get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                                     double default_value, double& value) const
{
  return read_or_declare(node_, full_name(plugin_name, parameter_name), default_value, value);
}

bool PluginParameters::get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                                     int64_t default_value, int64_t& value) const
{
  return read_or_declare(node_, full_name(plugin_name, parameter_name), default_value, value);
}

bool PluginParameters::get_parameter(const std::string& plugin_name, const std::string& parameter_name,
                                     const std::vector<double>& default_value, std::vector<double>& value) const
{
  return read_or_declare(node_, full_name(plugin_name, parameter_name), default_value, value);
}

std::string PluginParameters::full_name(const std::string& plugin_name, const std::string& parameter_name) const
{
  return "mujoco_plugins." + plugin_name + "." + parameter_name;
}

}  // namespace mujoco_ros2_control_plugins
