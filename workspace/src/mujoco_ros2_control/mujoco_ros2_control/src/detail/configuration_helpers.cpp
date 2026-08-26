/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/detail/configuration_helpers.hpp"

#include <filesystem>

#include <hardware_interface/lexical_casts.hpp>
#include <rcl/arguments.h>
#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control::detail
{
namespace
{

std::string trim_whitespace(std::string value)
{
  value.erase(0, value.find_first_not_of(" \t\n\r\f\v"));
  value.erase(value.find_last_not_of(" \t\n\r\f\v") + 1);
  return value;
}

}  // namespace

std::optional<std::string> get_hardware_parameter(const hardware_interface::HardwareInfo& hardware_info,
                                                  const std::string& key)
{
  if (auto it = hardware_info.hardware_parameters.find(key); it != hardware_info.hardware_parameters.end())
  {
    return it->second;
  }
  return std::nullopt;
}

std::string get_hardware_parameter_or(const hardware_interface::HardwareInfo& hardware_info, const std::string& key,
                                      const std::string& default_value)
{
  if (auto it = hardware_info.hardware_parameters.find(key); it != hardware_info.hardware_parameters.end())
  {
    return it->second;
  }
  return default_value;
}

std::optional<SimulationConfiguration> load_simulation_configuration(
    const hardware_interface::HardwareInfo& hardware_info, const rclcpp::Logger& logger)
{
  SimulationConfiguration configuration;

  const auto model_path_maybe = get_hardware_parameter(hardware_info, "mujoco_model");
  if (!model_path_maybe.has_value())
  {
    RCLCPP_INFO(logger, "Parameter 'mujoco_model' not found in URDF.");
    configuration.model_path.clear();
  }
  else
  {
    configuration.model_path = trim_whitespace(model_path_maybe.value());
    const std::filesystem::path path_to_file(configuration.model_path);
    if (!std::filesystem::exists(path_to_file))
    {
      RCLCPP_FATAL(logger, "MuJoCo model file '%s' does not exist!", configuration.model_path.c_str());
      return std::nullopt;
    }
    RCLCPP_INFO(logger, "Loading 'mujoco_model' from: '%s'", configuration.model_path.c_str());
  }

  configuration.speed_factor = std::stod(get_hardware_parameter(hardware_info, "sim_speed_factor").value_or("-1"));
  configuration.camera_publish_rate =
      std::stod(get_hardware_parameter(hardware_info, "camera_publish_rate").value_or("5.0"));
  configuration.headless =
      hardware_interface::parse_bool(get_hardware_parameter(hardware_info, "headless").value_or("false"));

  return configuration;
}

rclcpp::NodeOptions make_mujoco_node_options()
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("use_sim_time", rclcpp::ParameterValue(true));
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);
  return node_options;
}

bool validate_mujoco_joint_names(const mjModel* model, const rclcpp::Logger& logger)
{
  int num_joints_without_name = 0;
  for (int i = 0; i < model->njnt; ++i)
  {
    const char* joint_name = mj_id2name(model, mjtObj::mjOBJ_JOINT, i);
    const int joint_type = model->jnt_type[i];
    if (!joint_name && joint_type != mjJNT_FREE)
    {
      num_joints_without_name++;
    }
  }
  if (num_joints_without_name)
  {
    RCLCPP_FATAL(logger, "%d joints in the mjcf don't have names. All non-free joints must have names.",
                 num_joints_without_name);
    return false;
  }
  return true;
}

}  // namespace mujoco_ros2_control::detail
