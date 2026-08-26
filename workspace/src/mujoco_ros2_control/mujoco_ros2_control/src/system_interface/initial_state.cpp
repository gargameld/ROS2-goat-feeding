/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/system_interface/initial_state.hpp"

#include "mujoco_ros2_control/system_interface/joint_actuator_mapping.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{

void copy_passive_joint_states(std::vector<URDFJointData>& joints, std::vector<MuJoCoActuatorData>& actuators)
{
  for (auto& joint : joints)
  {
    std::for_each(actuators.begin(), actuators.end(), [&](auto& actuator) {
      if (actuator.joint_name == joint.name && actuator.actuator_type == ActuatorType::PASSIVE)
      {
        actuator.position_interface.state_ = joint.position_interface.state_;
        actuator.velocity_interface.state_ = joint.velocity_interface.state_;
        actuator.effort_interface.state_ = joint.effort_interface.state_;
      }
    });
  }
}

void apply_initial_joint_commands(std::vector<URDFJointData>& joints, std::vector<MuJoCoActuatorData>& actuators)
{
  if (joints.empty())
  {
    return;
  }

  // Transforms the joints' command to the actuator command interfaces
  copy_joint_commands_to_actuators(joints, actuators);

  // Set the initial actuator commands as actuator states
  std::for_each(actuators.begin(), actuators.end(),
                [](auto& actuator_interface) { actuator_interface.copy_command_to_state(); });

  copy_passive_joint_states(joints, actuators);
}

void apply_initial_pose(std::vector<MuJoCoActuatorData>& actuators, mjData* data, const rclcpp::Logger& logger)
{
  for (auto& actuator : actuators)
  {
    if (std::isfinite(actuator.position_interface.state_))
    {
      data->qpos[actuator.mj_pos_adr] = actuator.position_interface.state_;
    }
    else
    {
      RCLCPP_WARN_EXPRESSION(
          logger, actuator.actuator_type != ActuatorType::PASSIVE,
          "Actuator '%s' position state is not finite. Leaving it to the MuJoCo model's default initial position.",
          actuator.joint_name.c_str());
    }
    if (actuator.is_position_control_enabled)
    {
      data->ctrl[actuator.mj_actuator_id] = actuator.position_interface.state_;
    }
    else if (actuator.is_velocity_control_enabled)
    {
      data->ctrl[actuator.mj_actuator_id] = actuator.velocity_interface.state_;
    }
    else if (actuator.is_effort_control_enabled)
    {
      data->ctrl[actuator.mj_actuator_id] = actuator.effort_interface.state_;
    }
  }
}

}  // namespace mujoco_ros2_control
