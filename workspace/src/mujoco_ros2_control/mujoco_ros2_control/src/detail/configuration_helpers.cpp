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
  configuration.lidar_publish_rate =
      std::stod(get_hardware_parameter(hardware_info, "lidar_publish_rate").value_or("5.0"));
  configuration.headless =
      hardware_interface::parse_bool(get_hardware_parameter(hardware_info, "headless").value_or("false"));
  configuration.pids_config_file = get_hardware_parameter(hardware_info, "pids_config_file");
  configuration.model_topic =
      get_hardware_parameter_or(hardware_info, "mujoco_model_topic", "/mujoco_robot_description");

  return configuration;
}

std::optional<rclcpp::NodeOptions> make_mujoco_node_options(const SimulationConfiguration& configuration,
                                                           const rclcpp::Logger& logger)
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("use_sim_time", rclcpp::ParameterValue(true));
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);
  if (configuration.pids_config_file.has_value())
  {
    const std::string pids_config_file_path = trim_whitespace(configuration.pids_config_file.value());
    const std::filesystem::path path_to_file(pids_config_file_path);
    if (!std::filesystem::exists(path_to_file))
    {
      RCLCPP_FATAL(logger, "PID config file '%s' does not exist!", configuration.pids_config_file->c_str());
      return std::nullopt;
    }
    RCLCPP_INFO(logger, "Loading PID config from file: '%s'", configuration.pids_config_file.value().c_str());
    auto node_options_arguments = node_options.arguments();
    node_options_arguments.push_back(RCL_ROS_ARGS_FLAG);
    node_options_arguments.push_back(RCL_PARAM_FILE_FLAG);
    node_options_arguments.push_back(configuration.pids_config_file.value());
    node_options.arguments(node_options_arguments);
  }
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

FreeJointConfiguration find_free_joint(const mjModel* model, const std::string& joint_name,
                                       const rclcpp::Logger& logger)
{
  FreeJointConfiguration result;
  for (int i = 0; i < model->njnt; ++i)
  {
    const char* candidate_name = mj_id2name(model, mjtObj::mjOBJ_JOINT, i);
    if (candidate_name && joint_name == candidate_name)
    {
      if (model->jnt_type[i] == mjJNT_FREE)
      {
        result.joint_id = i;
        result.qpos_address = model->jnt_qposadr[i];
        result.qvel_address = model->jnt_dofadr[i];
      }
      else
      {
        RCLCPP_FATAL(logger, "Unable to use joint '%s' to publish the floating base state since it is not a free joint.",
                     joint_name.c_str());
        result.valid = false;
        return result;
      }
    }
  }
  return result;
}

}  // namespace mujoco_ros2_control::detail
