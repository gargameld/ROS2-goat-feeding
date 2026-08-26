/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>

namespace mujoco_ros2_control
{

/**
 * @brief Check that the compiled MJCF can be addressed by ros2_control.
 *
 * Every non-free joint must carry a name, because joints are matched to URDF joints by name.
 * @return false, after logging which joints are unnamed, when the model cannot be used.
 */
bool validate_mujoco_joint_names(const mjModel* model, const rclcpp::Logger& logger);

}  // namespace mujoco_ros2_control
