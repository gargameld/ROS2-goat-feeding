/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/detail/initial_state_helpers.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control::detail
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

void reset_actuator_interfaces(std::vector<MuJoCoActuatorData>& actuators, const mjData* data, mjData* control_data)
{
  for (auto& actuator : actuators)
  {
    actuator.position_interface.state_ = data->qpos[actuator.mj_pos_adr];
    actuator.velocity_interface.state_ = data->qvel[actuator.mj_vel_adr];
    actuator.effort_interface.state_ = 0.0;

    if (actuator.actuator_type != ActuatorType::PASSIVE)
    {
      actuator.is_position_control_enabled = actuator.actuator_type == ActuatorType::POSITION;
      actuator.is_velocity_control_enabled = actuator.actuator_type == ActuatorType::VELOCITY;
      actuator.is_effort_control_enabled =
          actuator.actuator_type == ActuatorType::MOTOR || actuator.actuator_type == ActuatorType::CUSTOM;
      actuator.position_interface.command_ = actuator.position_interface.state_;
      actuator.velocity_interface.command_ = 0.0;
      actuator.effort_interface.command_ = 0.0;

      if (actuator.is_position_control_enabled && actuator.mj_actuator_id >= 0)
      {
        control_data->ctrl[actuator.mj_actuator_id] = actuator.position_interface.state_;
      }
    }
  }
}

void reset_joint_commands(std::vector<URDFJointData>& joints)
{
  for (auto& joint : joints)
  {
    joint.position_interface.command_ = joint.position_interface.state_;
    joint.velocity_interface.command_ = 0.0;
    joint.effort_interface.command_ = 0.0;
  }
}

}  // namespace mujoco_ros2_control::detail
