/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>

#include "mujoco_ros2_control/data.hpp"

namespace mujoco_ros2_control
{

void update_joint_control_mode(const std::string& interface_name, bool enabled,
                               std::vector<URDFJointData>& joints, std::vector<MuJoCoActuatorData>& actuators,
                               const rclcpp::Logger& logger);

}  // namespace mujoco_ros2_control
