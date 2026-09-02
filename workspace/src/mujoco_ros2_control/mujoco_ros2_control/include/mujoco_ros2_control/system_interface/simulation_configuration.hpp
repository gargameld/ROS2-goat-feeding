
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
