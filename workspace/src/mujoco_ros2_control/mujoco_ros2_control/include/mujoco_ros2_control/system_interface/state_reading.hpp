/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <vector>

#include <mujoco/mujoco.h>

#include "mujoco_ros2_control/data.hpp"

namespace mujoco_ros2_control
{

void read_actuator_states(const mjData* control_data, std::vector<MuJoCoActuatorData>& actuators);

void read_imu_states(const mjData* control_data, std::vector<IMUSensorData>& sensors);

}  // namespace mujoco_ros2_control
