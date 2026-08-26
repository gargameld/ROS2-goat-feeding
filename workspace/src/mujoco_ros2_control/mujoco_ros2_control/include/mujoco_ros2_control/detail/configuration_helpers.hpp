/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <optional>
#include <string>

#include <hardware_interface/hardware_info.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_options.hpp>

namespace mujoco_ros2_control::detail
{

struct SimulationConfiguration
{
  std::string model_path;
  double speed_factor;
  double camera_publish_rate;
  bool headless;
};

std::optional<std::string> get_hardware_parameter(const hardware_interface::HardwareInfo& hardware_info,
                                                  const std::string& key);

std::string get_hardware_parameter_or(const hardware_interface::HardwareInfo& hardware_info, const std::string& key,
                                      const std::string& default_value);

std::optional<SimulationConfiguration> load_simulation_configuration(
    const hardware_interface::HardwareInfo& hardware_info, const rclcpp::Logger& logger);

rclcpp::NodeOptions make_mujoco_node_options();

bool validate_mujoco_joint_names(const mjModel* model, const rclcpp::Logger& logger);

}  // namespace mujoco_ros2_control::detail
