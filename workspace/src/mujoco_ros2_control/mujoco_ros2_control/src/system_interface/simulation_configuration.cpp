/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/system_interface/simulation_configuration.hpp"

#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control/hardware_parameters.hpp"

namespace mujoco_ros2_control
{
namespace
{

constexpr double default_camera_publish_rate_hz = 5.0;

}  // namespace

std::optional<SimulationConfiguration> load_simulation_configuration(
    const hardware_interface::HardwareInfo& hardware_info, const rclcpp::Logger& logger)
{
  const HardwareParameters parameters(hardware_info);
  SimulationConfiguration configuration;

  const auto model_path_maybe = parameters.find("mujoco_model");
  if (!model_path_maybe.has_value())
  {
    RCLCPP_INFO(logger, "Parameter 'mujoco_model' not found in URDF.");
    configuration.model_path.clear();
  }
  else
  {
    configuration.model_path = model_path_maybe.value();
    const std::filesystem::path path_to_file(configuration.model_path);
    if (!std::filesystem::exists(path_to_file))
    {
      RCLCPP_FATAL(logger, "MuJoCo model file '%s' does not exist!", configuration.model_path.c_str());
      return std::nullopt;
    }
    RCLCPP_INFO(logger, "Loading 'mujoco_model' from: '%s'", configuration.model_path.c_str());
  }

  configuration.camera_publish_rate =
      parameters.get_positive_double("camera_publish_rate", default_camera_publish_rate_hz);

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

}  // namespace mujoco_ros2_control
