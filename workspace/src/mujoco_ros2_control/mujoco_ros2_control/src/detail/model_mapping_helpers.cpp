/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/detail/model_mapping_helpers.hpp"

#include <regex>

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control::detail
{

bool is_mimic_joint(const std::string& joint_name, const hardware_interface::HardwareInfo& hardware_info)
{
  for (const auto& joint : hardware_info.joints)
  {
    if (joint.parameters.find("mimic") != joint.parameters.end() && joint.name == joint_name)
    {
      return true;
    }
  }
  return false;
}

ActuatorType get_actuator_type(const mjModel* mj_model, int mujoco_actuator_id)
{
  ActuatorType actuator_type = ActuatorType::UNKNOWN;
  int biastype = mj_model->actuator_biastype[mujoco_actuator_id];
  const int NBias = 10;
  const mjtNum* biasprm = mj_model->actuator_biasprm + mujoco_actuator_id * NBias;

  if (biastype == mjBIAS_NONE)
  {
    actuator_type = ActuatorType::MOTOR;
  }
  else if (biastype == mjBIAS_AFFINE && biasprm[1] != 0)
  {
    actuator_type = ActuatorType::POSITION;
  }
  else if (biastype == mjBIAS_AFFINE && biasprm[1] == 0 && biasprm[2] != 0)
  {
    actuator_type = ActuatorType::VELOCITY;
  }
  else
  {
    actuator_type = ActuatorType::CUSTOM;
  }

  return actuator_type;
}

int get_actuator_id(const std::string& actuator_name, const mjModel* mj_model)
{
  int mujoco_actuator_id = mj_name2id(mj_model, mjtObj::mjOBJ_JOINT, actuator_name.c_str());
  if (mujoco_actuator_id == -1)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("MujocoSystemInterface"), "Failed to find the actuator : '%s' in the MuJoCo model",
                 actuator_name.c_str());
  }

  for (int i = 0; i < mj_model->nu; ++i)
  {
    if (mj_model->actuator_trntype[i] == mjTRN_JOINT && mj_model->actuator_trnid[2 * i] == mujoco_actuator_id)
    {
      mujoco_actuator_id = i;
      break;
    }
  }

  mujoco_actuator_id = mujoco_actuator_id == -1 ? mj_name2id(mj_model, mjtObj::mjOBJ_ACTUATOR, actuator_name.c_str()) :
                                                  mujoco_actuator_id;
  return mujoco_actuator_id;
}

std::string get_joint_actuator_name(const std::string& joint_name,
                                    const hardware_interface::HardwareInfo& hardware_info, const mjModel* mj_model)
{
  std::string actuator_name = joint_name;

  for (const auto& transmission : hardware_info.transmissions)
  {
    for (const auto& joint : transmission.joints)
    {
      if (joint.name == joint_name)
      {
        if (get_actuator_id(joint_name, mj_model) != -1)
        {
          return joint_name;
        }
        const std::string corresponding_actuator_role = std::regex_replace(joint.role, std::regex("joint"), "actuator");
        for (const auto& actuator : transmission.actuators)
        {
          if (actuator.role == corresponding_actuator_role)
          {
            RCLCPP_DEBUG(rclcpp::get_logger("MujocoSystemInterface"),
                         "Mapped joint '%s' to actuator '%s' based on role '%s'", joint_name.c_str(),
                         actuator.name.c_str(), corresponding_actuator_role.c_str());
            return actuator.name;
          }
        }
        RCLCPP_WARN(rclcpp::get_logger("MujocoSystemInterface"),
                    "No matching actuator found for joint '%s' with role '%s'. Using joint name as actuator name.",
                    joint_name.c_str(), joint.role.c_str());
        break;
      }
    }
  }

  return actuator_name;
}

std::vector<std::string> get_interfaces_in_order(const std::vector<std::string>& available_interfaces,
                                                 const std::vector<std::string>& desired_order)
{
  std::vector<std::string> ordered_interfaces;
  for (const auto& interface : desired_order)
  {
    if (std::find(available_interfaces.begin(), available_interfaces.end(), interface) != available_interfaces.end())
    {
      ordered_interfaces.push_back(interface);
    }
  }
  add_items(ordered_interfaces, available_interfaces);
  return ordered_interfaces;
}

}  // namespace mujoco_ros2_control::detail
