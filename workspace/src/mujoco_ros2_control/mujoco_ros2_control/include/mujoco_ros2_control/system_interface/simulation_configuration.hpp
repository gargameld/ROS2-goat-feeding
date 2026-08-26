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
#include <rclcpp/logger.hpp>
#include <rclcpp/node_options.hpp>

namespace mujoco_ros2_control
{

/// Simulation settings read from the `<hardware>` parameters in the ros2_control URDF.
struct SimulationConfiguration
{
  std::string model_path;
  double camera_publish_rate;
};

/**
 * @brief Look up one `<param>` from the ros2_control `<hardware>` block.
 */
std::optional<std::string> get_hardware_parameter(const hardware_interface::HardwareInfo& hardware_info,
                                                  const std::string& key);

/**
 * @brief Read the simulation settings out of the hardware parameters.
 * @return std::nullopt when `mujoco_model` names a file that does not exist.
 */
std::optional<SimulationConfiguration> load_simulation_configuration(
    const hardware_interface::HardwareInfo& hardware_info, const rclcpp::Logger& logger);

/**
 * @brief Node options for the simulation's node: sim time on, parameters declared from overrides.
 */
rclcpp::NodeOptions make_mujoco_node_options();

}  // namespace mujoco_ros2_control
