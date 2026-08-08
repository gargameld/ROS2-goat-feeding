/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/detail/control_mode_helpers.hpp"

#include <algorithm>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control/detail/model_mapping_helpers.hpp"

namespace mujoco_ros2_control::detail
{

void update_joint_control_mode(const std::string& interface_name, bool enabled,
                               const hardware_interface::HardwareInfo& hardware_info, const mjModel* model,
                               std::vector<URDFJointData>& joints, std::vector<MuJoCoActuatorData>& actuators,
                               const rclcpp::Logger& logger)
{
  const size_t delimiter_pos = interface_name.rfind('/');
  if (delimiter_pos == std::string::npos)
  {
    RCLCPP_ERROR(logger, "Invalid interface name format: %s", interface_name.c_str());
    return;
  }

  std::string joint_name = interface_name.substr(0, delimiter_pos);
  std::string interface_type = interface_name.substr(delimiter_pos + 1);

  auto joint_it = std::find_if(joints.begin(), joints.end(),
                               [&joint_name](const URDFJointData& joint) { return joint.name == joint_name; });
  if (joint_it == joints.end())
  {
    RCLCPP_WARN(logger, "Joint %s not found in urdf_joint_data_", joint_name.c_str());
    return;
  }

  const auto actuator_name = get_joint_actuator_name(joint_name, hardware_info, model);
  auto actuator_it =
      std::find_if(actuators.begin(), actuators.end(), [&actuator_name](const MuJoCoActuatorData& actuator) {
        return actuator.joint_name == actuator_name;
      });
  if (actuator_it == actuators.end())
  {
    RCLCPP_WARN(logger, "Actuator %s not found in mujoco_actuator_data_", actuator_name.c_str());
    return;
  }
  if (actuator_it->actuator_type == ActuatorType::PASSIVE)
  {
    RCLCPP_WARN(logger, "Actuator %s is passive and cannot be controlled.", actuator_name.c_str());
    return;
  }

  if (enabled)
  {
    joint_it->is_position_control_enabled = false;
    joint_it->is_velocity_control_enabled = false;
    joint_it->is_effort_control_enabled = false;

    actuator_it->is_position_control_enabled = false;
    actuator_it->is_velocity_control_enabled = false;
    actuator_it->is_effort_control_enabled = false;
    actuator_it->is_position_pid_control_enabled = false;
    actuator_it->is_velocity_pid_control_enabled = false;

    if (interface_type == hardware_interface::HW_IF_POSITION)
    {
      actuator_it->is_position_control_enabled = (actuator_it->pos_pid == nullptr);
      actuator_it->is_position_pid_control_enabled = (actuator_it->pos_pid != nullptr);
      joint_it->is_position_control_enabled = true;
      RCLCPP_INFO(logger, "Joint %s: position control enabled (velocity, effort disabled)", joint_name.c_str());
    }
    else if (interface_type == hardware_interface::HW_IF_VELOCITY)
    {
      actuator_it->is_velocity_control_enabled = (actuator_it->vel_pid == nullptr);
      actuator_it->is_velocity_pid_control_enabled = (actuator_it->vel_pid != nullptr);
      joint_it->is_velocity_control_enabled = true;
      RCLCPP_INFO(logger, "Joint %s: velocity control enabled (position, effort disabled)", joint_name.c_str());
    }
    else if (interface_type == hardware_interface::HW_IF_EFFORT || interface_type == hardware_interface::HW_IF_TORQUE ||
             interface_type == hardware_interface::HW_IF_FORCE)
    {
      actuator_it->is_effort_control_enabled = true;
      joint_it->is_effort_control_enabled = true;
      RCLCPP_INFO(logger, "Joint %s: %s control enabled (position, velocity disabled)", joint_name.c_str(),
                  interface_type.c_str());
    }
  }
  else
  {
    joint_it->is_position_control_enabled = false;
    joint_it->is_velocity_control_enabled = false;
    joint_it->is_effort_control_enabled = false;

    actuator_it->is_position_control_enabled = false;
    actuator_it->is_velocity_control_enabled = false;
    actuator_it->is_effort_control_enabled = false;
    actuator_it->is_position_pid_control_enabled = false;
    actuator_it->is_velocity_pid_control_enabled = false;

    RCLCPP_INFO(logger, "Joint %s: %s control disabled", joint_name.c_str(), interface_type.c_str());
  }
}

}  // namespace mujoco_ros2_control::detail
