
#pragma once

#include <hardware_interface/hardware_info.hpp>

namespace mujoco_ros2_control
{

/**
 * @brief Returns the sensor's component info for the provided sensor name, if it exists.
 */
inline std::optional<hardware_interface::ComponentInfo>
get_sensor_from_info(const hardware_interface::HardwareInfo& hardware_info, const std::string& name)
{
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    const auto& sensor = hardware_info.sensors.at(sensor_index);
    if (hardware_info.sensors.at(sensor_index).name == name)
    {
      return sensor;
    }
  }
  return std::nullopt;
}

}  // namespace mujoco_ros2_control
