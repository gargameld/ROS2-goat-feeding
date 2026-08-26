/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/system_interface/mujoco_actuator_discovery.hpp"

#include <algorithm>

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{

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

bool map_actuator_to_joint(const mjModel* model, int actuator_id, MuJoCoActuatorData& actuator_data,
                           const rclcpp::Logger& logger)
{
  int transmission_type = model->actuator_trntype[actuator_id];
  int target_id = model->actuator_trnid[actuator_id * 2];
  const char* actuator_name = mj_id2name(model, mjOBJ_ACTUATOR, actuator_id);
  if (!actuator_name)
  {
    actuator_name = "unnamed";
  }

  if (transmission_type == mjTRN_JOINT)
  {
    const char* joint_name = mj_id2name(model, mjOBJ_JOINT, target_id);
    if (joint_name)
    {
      actuator_data.joint_name = std::string(joint_name);
      RCLCPP_INFO(logger, "Registering MuJoCo actuator '%s' for joint '%s'", actuator_data.joint_name.c_str(),
                  joint_name);
    }
    else
    {
      RCLCPP_ERROR(logger, "Failed to find joint name for actuator '%s'", actuator_name);
      return false;
    }
  }
  else if (transmission_type == mjTRN_TENDON)
  {
    int joint_id = mj_name2id(model, mjOBJ_JOINT, actuator_name);
    if (joint_id != -1)
    {
      target_id = joint_id;
      actuator_data.joint_name = std::string(actuator_name);
      RCLCPP_INFO(logger, "Registering MuJoCo tendon actuator '%s' using joint state", actuator_name);
    }
    else
    {
      RCLCPP_ERROR(logger,
                   "Tendon actuator '%s' has no matching joint. Tendon actuators must be named the same as a joint "
                   "that they will control.",
                   actuator_name);
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Unsupported transmission type '%d' for actuator '%s'", transmission_type, actuator_name);
    return false;
  }

  actuator_data.mj_actuator_id = actuator_id;
  actuator_data.mj_pos_adr = model->jnt_qposadr[target_id];
  actuator_data.mj_vel_adr = model->jnt_dofadr[target_id];
  actuator_data.actuator_type = get_actuator_type(model, actuator_data.mj_actuator_id);
  return true;
}

void initialize_actuator_control(MuJoCoActuatorData& actuator_data)
{
  if (actuator_data.actuator_type == ActuatorType::POSITION)
  {
    actuator_data.is_position_control_enabled = true;
  }
  else if (actuator_data.actuator_type == ActuatorType::VELOCITY)
  {
    actuator_data.is_velocity_control_enabled = true;
  }
  else if (actuator_data.actuator_type == ActuatorType::MOTOR || actuator_data.actuator_type == ActuatorType::CUSTOM)
  {
    actuator_data.is_effort_control_enabled = true;
  }
}

void append_passive_actuators(const mjModel* model, std::vector<MuJoCoActuatorData>& actuators,
                              const rclcpp::Logger& logger)
{
  for (int joint_id = 0; joint_id < model->njnt; joint_id++)
  {
    const auto actuator_it =
        std::find_if(actuators.cbegin(), actuators.cend(), [model, joint_id](const MuJoCoActuatorData& actuator) {
          return actuator.mj_pos_adr == model->jnt_qposadr[joint_id];
        });
    if (actuator_it == actuators.cend() && model->jnt_type[joint_id] != mjJNT_FREE &&
        model->jnt_type[joint_id] != mjJNT_BALL)
    {
      MuJoCoActuatorData passive_actuator;
      passive_actuator.joint_name = std::string(mj_id2name(model, mjOBJ_JOINT, joint_id));
      RCLCPP_INFO(logger, "MuJoCo joint '%s' has no associated actuator. Registering as a passive joint.",
                  passive_actuator.joint_name.c_str());
      passive_actuator.mj_pos_adr = model->jnt_qposadr[joint_id];
      passive_actuator.mj_vel_adr = model->jnt_dofadr[joint_id];
      passive_actuator.actuator_type = ActuatorType::PASSIVE;
      actuators.push_back(passive_actuator);
    }
  }
}

bool discover_mujoco_actuators(const mjModel* model, std::vector<MuJoCoActuatorData>& actuators,
                               const rclcpp::Logger& logger)
{
  actuators.clear();
  actuators.resize(model->nu);

  for (int i = 0; i < model->nu; i++)
  {
    RCLCPP_DEBUG(logger, "Registering MuJoCo actuator %ld/%ld", static_cast<long>(i + 1),
                 static_cast<long>(model->nu));
    MuJoCoActuatorData& actuator_data = actuators.at(i);
    if (!map_actuator_to_joint(model, i, actuator_data, logger))
    {
      return false;
    }
    initialize_actuator_control(actuator_data);

    const char* act_name = mj_id2name(model, mjOBJ_ACTUATOR, i);
    if (!act_name)
    {
      act_name = "unnamed";
    }
    RCLCPP_DEBUG(logger, "Successfully registered actuator '%s'", act_name);
  }

  // now look out for the MuJoCo joints that do not have any actuator associated with them
  append_passive_actuators(model, actuators, logger);

  return true;
}

MuJoCoActuatorData* find_controllable_actuator(std::vector<MuJoCoActuatorData>& actuators, const mjModel* model,
                                               const std::string& actuator_name)
{
  const auto actuator_it =
      std::find_if(actuators.begin(), actuators.end(), [&actuator_name, model](const MuJoCoActuatorData& actuator) {
        return (actuator.actuator_type != ActuatorType::PASSIVE) &&
               ((mj_id2name(model, mjOBJ_ACTUATOR, actuator.mj_actuator_id) == actuator_name) ||
                (actuator.joint_name == actuator_name));
      });
  return actuator_it == actuators.end() ? nullptr : &*actuator_it;
}

}  // namespace mujoco_ros2_control
