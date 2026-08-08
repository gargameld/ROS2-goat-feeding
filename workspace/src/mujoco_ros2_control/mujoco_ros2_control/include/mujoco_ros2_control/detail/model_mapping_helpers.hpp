/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/version.h>
#include <mujoco/mujoco.h>

#if HARDWARE_INTERFACE_VERSION_MAJOR >= 3
#include <hardware_interface/helpers.hpp>
#endif

#include "mujoco_ros2_control/data.hpp"

namespace mujoco_ros2_control::detail
{

template <typename T>
void add_items(std::vector<T>& destination, const std::vector<T>& items)
{
#if HARDWARE_INTERFACE_VERSION_MAJOR < 3
  for (const auto& item : items)
  {
    if (std::find(destination.begin(), destination.end(), item) == destination.end())
    {
      destination.push_back(item);
    }
  }
#else
  for (const auto& item : items)
  {
    ros2_control::add_item(destination, item);
  }
#endif
}

bool is_mimic_joint(const std::string& joint_name, const hardware_interface::HardwareInfo& hardware_info);

ActuatorType get_actuator_type(const mjModel* mj_model, int mujoco_actuator_id);

int get_actuator_id(const std::string& actuator_name, const mjModel* mj_model);

std::string get_joint_actuator_name(const std::string& joint_name,
                                    const hardware_interface::HardwareInfo& hardware_info, const mjModel* mj_model);

std::vector<std::string> get_interfaces_in_order(const std::vector<std::string>& available_interfaces,
                                                 const std::vector<std::string>& desired_order);

}  // namespace mujoco_ros2_control::detail
