/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <string>
#include <unordered_map>

#include <hardware_interface/hardware_info.hpp>

namespace mujoco_ros2_control
{

/// ros2_control component descriptions, keyed by the component's name.
using ComponentInfoMap = std::unordered_map<std::string, hardware_interface::ComponentInfo>;

}  // namespace mujoco_ros2_control
