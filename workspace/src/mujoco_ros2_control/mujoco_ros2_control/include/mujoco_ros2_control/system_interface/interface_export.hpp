/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <vector>

#include <hardware_interface/handle.hpp>

#include "mujoco_ros2_control/data.hpp"
#include "mujoco_ros2_control/system_interface/component_info_map.hpp"

namespace mujoco_ros2_control
{

void append_joint_state_interfaces(std::vector<hardware_interface::StateInterface>& interfaces,
                                   std::vector<URDFJointData>& joints, const ComponentInfoMap& joint_hardware_info);

void append_imu_state_interfaces(std::vector<hardware_interface::StateInterface>& interfaces,
                                 std::vector<IMUSensorData>& sensors, const ComponentInfoMap& sensor_hardware_info);

void append_joint_command_interfaces(std::vector<hardware_interface::CommandInterface>& interfaces,
                                     std::vector<URDFJointData>& joints,
                                     const ComponentInfoMap& joint_hardware_info);

}  // namespace mujoco_ros2_control
