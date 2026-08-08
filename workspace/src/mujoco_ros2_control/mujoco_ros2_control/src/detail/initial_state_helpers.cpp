/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/detail/initial_state_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <tinyxml2.h>

namespace mujoco_ros2_control::detail
{

std::optional<InitialStateValues> load_initial_state_values(const std::string& file_path,
                                                            const rclcpp::Logger& logger)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS)
  {
    RCLCPP_ERROR(logger, "Failed to load override start position file : '%s'.", file_path.c_str());
    return std::nullopt;
  }

  tinyxml2::XMLElement* key_element = document.FirstChildElement("key");
  if (!key_element)
  {
    RCLCPP_ERROR(logger, "<key> element not found in override start position file.");
    return std::nullopt;
  }

  auto parse_attribute = [&](const char* attribute_name) -> std::vector<double> {
    std::vector<double> result;
    const char* text = key_element->Attribute(attribute_name);
    if (!text)
    {
      RCLCPP_ERROR(logger, "Attribute '%s' not found in override start position file.", attribute_name);
      return result;
    }

    std::stringstream stream(text);
    double value;
    while (stream >> value)
    {
      result.push_back(value);
    }
    return result;
  };

  InitialStateValues values{ parse_attribute("qpos"), parse_attribute("qvel"), parse_attribute("ctrl") };
  if (values.qpos.empty() || values.qvel.empty() || values.ctrl.empty())
  {
    return std::nullopt;
  }
  return values;
}

bool initial_state_sizes_match(const InitialStateValues& values, const mjModel* model, const rclcpp::Logger& logger)
{
  if ((values.qpos.size() != static_cast<size_t>(model->nq)) ||
      (values.qvel.size() != static_cast<size_t>(model->nv)) ||
      (values.ctrl.size() != static_cast<size_t>(model->nu)))
  {
    RCLCPP_ERROR(logger,
                 "Mismatch in data types in override starting positions. Numbers are:\n\t"
                 "qpos size in file: %zu, qpos size in model: %d\n\t"
                 "qvel size in file: %zu, qvel size in model: %d\n\t"
                 "ctrl size in file: %zu, ctrl size in model: %d",
                 values.qpos.size(), model->nq, values.qvel.size(), model->nv, values.ctrl.size(), model->nu);
    return false;
  }
  return true;
}

void copy_initial_state_to_data(const InitialStateValues& values, mjData* data)
{
  std::copy(values.qpos.begin(), values.qpos.end(), data->qpos);
  std::copy(values.qvel.begin(), values.qvel.end(), data->qvel);
  std::copy(values.ctrl.begin(), values.ctrl.end(), data->ctrl);
}

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

    if (actuator.pos_pid)
    {
      actuator.pos_pid->reset();
    }
    if (actuator.vel_pid)
    {
      actuator.vel_pid->reset();
    }

    if (actuator.actuator_type != ActuatorType::PASSIVE)
    {
      actuator.is_position_pid_control_enabled = actuator.has_pos_pid;
      actuator.is_position_control_enabled = !actuator.has_pos_pid && actuator.actuator_type == ActuatorType::POSITION;
      actuator.is_velocity_pid_control_enabled = !actuator.has_pos_pid && actuator.has_vel_pid;
      actuator.is_velocity_control_enabled =
          !actuator.has_pos_pid && !actuator.has_vel_pid && actuator.actuator_type == ActuatorType::VELOCITY;
      actuator.is_effort_control_enabled =
          !actuator.has_pos_pid && !actuator.has_vel_pid &&
          (actuator.actuator_type == ActuatorType::MOTOR || actuator.actuator_type == ActuatorType::CUSTOM);
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
