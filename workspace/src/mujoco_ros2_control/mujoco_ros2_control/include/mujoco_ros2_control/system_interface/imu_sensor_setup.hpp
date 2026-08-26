/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <optional>
#include <string>

#include <hardware_interface/hardware_info.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>

#include "mujoco_ros2_control/data.hpp"

namespace mujoco_ros2_control
{

/**
 * @brief Locate the three MJCF sensors backing one ros2_control IMU and record their addresses.
 *
 * MuJoCo has no single IMU sensor, so an IMU named `<IMU>` is expected to appear in the MJCF as
 * three separate sensors, all on the same site:
 *
 *  <sensor>
 *    <framequat name="<IMU>_quat" objtype="site" objname="obj_imu" />
 *    <gyro name="<IMU>_gyro" site="obj_imu" />
 *    <accelerometer name="<IMU>_accel" site="obj_imu" />
 *  </sensor>
 *
 * These are mapped with the following xacro (note the `_imu` suffix):
 *
 *  <sensor name="<IMU>_imu">
 *    <state_interface name="orientation.x"/>
 *    <state_interface name="orientation.y"/>
 *    <state_interface name="orientation.z"/>
 *    <state_interface name="orientation.w"/>
 *    <state_interface name="angular_velocity.x"/>
 *    <state_interface name="angular_velocity.y"/>
 *    <state_interface name="angular_velocity.z"/>
 *    <state_interface name="linear_acceleration.x"/>
 *    <state_interface name="linear_acceleration.y"/>
 *    <state_interface name="linear_acceleration.z"/>
 *  </sensor>
 *
 * @return std::nullopt when any of the three MJCF sensors is missing.
 */
std::optional<IMUSensorData> make_imu_sensor(const hardware_interface::ComponentInfo& sensor,
                                             const std::string& mujoco_sensor_name, const mjModel* model,
                                             const rclcpp::Logger& logger);

}  // namespace mujoco_ros2_control
