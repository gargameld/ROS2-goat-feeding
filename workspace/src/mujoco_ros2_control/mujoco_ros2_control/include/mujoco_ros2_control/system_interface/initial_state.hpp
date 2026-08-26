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

namespace mujoco_ros2_control
{

void copy_passive_joint_states(std::vector<URDFJointData>& joints, std::vector<MuJoCoActuatorData>& actuators);

/**
 * @brief Seed the actuator command and state interfaces from the URDF's initial joint values.
 *
 * The joints already carry their initial values, this hands them to the actuators as commands,
 * mirrors those commands into the actuator states, and copies passive joint states across.
 */
void apply_initial_joint_commands(std::vector<URDFJointData>& joints, std::vector<MuJoCoActuatorData>& actuators);

void apply_initial_pose(std::vector<MuJoCoActuatorData>& actuators, mjData* data, const rclcpp::Logger& logger);

}  // namespace mujoco_ros2_control
