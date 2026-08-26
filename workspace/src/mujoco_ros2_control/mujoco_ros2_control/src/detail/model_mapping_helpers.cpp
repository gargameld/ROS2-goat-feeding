/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/detail/model_mapping_helpers.hpp"


#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control::detail
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
