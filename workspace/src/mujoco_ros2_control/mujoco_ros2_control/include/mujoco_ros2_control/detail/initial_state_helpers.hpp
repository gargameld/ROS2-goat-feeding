/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>

#include "mujoco_ros2_control/data.hpp"

namespace mujoco_ros2_control::detail
{

void copy_passive_joint_states(std::vector<URDFJointData>& joints, std::vector<MuJoCoActuatorData>& actuators);

void apply_initial_pose(std::vector<MuJoCoActuatorData>& actuators, mjData* data, const rclcpp::Logger& logger);

void reset_actuator_interfaces(std::vector<MuJoCoActuatorData>& actuators, const mjData* data, mjData* control_data);

void reset_joint_commands(std::vector<URDFJointData>& joints);

}  // namespace mujoco_ros2_control::detail
