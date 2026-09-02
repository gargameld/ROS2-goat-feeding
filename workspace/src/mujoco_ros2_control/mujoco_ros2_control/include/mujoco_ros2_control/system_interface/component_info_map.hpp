

#pragma once

#include <string>
#include <unordered_map>

#include <hardware_interface/hardware_info.hpp>

namespace mujoco_ros2_control
{

/// ros2_control component descriptions, keyed by the component's name.
using ComponentInfoMap = std::unordered_map<std::string, hardware_interface::ComponentInfo>;

}  // namespace mujoco_ros2_control
