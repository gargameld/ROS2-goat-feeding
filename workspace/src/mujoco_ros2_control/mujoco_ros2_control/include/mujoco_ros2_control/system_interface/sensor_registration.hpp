/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>

#include "mujoco_ros2_control/data.hpp"
#include "mujoco_ros2_control/system_interface/component_info_map.hpp"

namespace mujoco_ros2_control
{

/**
 * @brief Construct all sensor data containers described by the ros2_control hardware info.
 *
 * A sensor is only backed by the simulation when it carries a `mujoco_type` parameter; the others
 * are left to whichever hardware interface owns them. The MJCF sensor it maps to is named by the
 * `mujoco_sensor_name` parameter, defaulting to the ros2_control sensor's own name.
 *
 * IMU is currently the only supported `mujoco_type`; see make_imu_sensor() for the MJCF sensors
 * one IMU is expected to be built from.
 *
 * @param[out] sensor_hardware_info Hardware info of every simulated sensor, keyed by sensor name.
 * @param[out] imu_sensors Data containers for the registered IMU sensors.
 */
void register_sensors(const hardware_interface::HardwareInfo& hardware_info, const mjModel* model,
                      ComponentInfoMap& sensor_hardware_info, std::vector<IMUSensorData>& imu_sensors,
                      const rclcpp::Logger& logger);

}  // namespace mujoco_ros2_control
