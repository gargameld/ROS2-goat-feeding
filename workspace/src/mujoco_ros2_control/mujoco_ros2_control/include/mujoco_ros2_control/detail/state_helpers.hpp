/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <vector>

#include <mujoco/mujoco.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include "mujoco_ros2_control/data.hpp"

namespace mujoco_ros2_control::detail
{

void read_actuator_states(const mjData* control_data, std::vector<MuJoCoActuatorData>& actuators,
                          sensor_msgs::msg::JointState& actuator_state_message);

void read_imu_states(const mjData* control_data, std::vector<IMUSensorData>& sensors);

}  // namespace mujoco_ros2_control::detail
