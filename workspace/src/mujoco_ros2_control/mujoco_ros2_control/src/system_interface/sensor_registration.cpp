/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/system_interface/sensor_registration.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control/hardware_parameters.hpp"
#include "mujoco_ros2_control/system_interface/imu_sensor_setup.hpp"

namespace mujoco_ros2_control
{

void register_sensors(const hardware_interface::HardwareInfo& hardware_info, const mjModel* model,
                      ComponentInfoMap& sensor_hardware_info, std::vector<IMUSensorData>& imu_sensors,
                      const rclcpp::Logger& logger)
{
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    auto sensor = hardware_info.sensors.at(sensor_index);
    const std::string sensor_name = sensor.name;

    const HardwareParameters sensor_parameters(sensor);

    const auto mujoco_type_maybe = sensor_parameters.find("mujoco_type");
    if (!mujoco_type_maybe.has_value())
    {
      RCLCPP_INFO(logger, "Not adding hardware interface for sensor in ros2_control xacro: '%s'", sensor_name.c_str());
      continue;
    }
    const auto mujoco_type = mujoco_type_maybe.value();

    // If there is a specific sensor name provided we use that, otherwise we assume the MuJoCo model's
    // sensor is named identically to the ros2_control hardware interface's.
    const std::string mujoco_sensor_name = sensor_parameters.get_string("mujoco_sensor_name", sensor_name);

    RCLCPP_INFO(logger, "Adding sensor named: '%s', of type: '%s', mapping to the MJCF sensor: '%s'",
                sensor_name.c_str(), mujoco_type.c_str(), mujoco_sensor_name.c_str());

    // Add to the sensor hw information map
    sensor_hardware_info.insert(std::make_pair(sensor_name, sensor));

    if (mujoco_type == "imu")
    {
      const auto sensor_data = make_imu_sensor(sensor, mujoco_sensor_name, model, logger);
      if (sensor_data.has_value())
      {
        imu_sensors.push_back(sensor_data.value());
      }
    }
    else
    {
      RCLCPP_ERROR(logger, "Invalid mujoco_type passed to the MuJoCo hardware interface: '%s'", mujoco_type.c_str());
    }
  }
}

}  // namespace mujoco_ros2_control
