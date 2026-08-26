/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/system_interface/imu_sensor_setup.hpp"

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{

std::optional<IMUSensorData> make_imu_sensor(const hardware_interface::ComponentInfo& sensor,
                                             const std::string& mujoco_sensor_name, const mjModel* model,
                                             const rclcpp::Logger& logger)
{
  IMUSensorData sensor_data;
  sensor_data.name = sensor.name;
  sensor_data.orientation.name = mujoco_sensor_name + "_quat";
  sensor_data.angular_velocity.name = mujoco_sensor_name + "_gyro";
  sensor_data.linear_acceleration.name = mujoco_sensor_name + "_accel";

  const int quaternion_id = mj_name2id(model, mjOBJ_SENSOR, sensor_data.orientation.name.c_str());
  const int gyroscope_id = mj_name2id(model, mjOBJ_SENSOR, sensor_data.angular_velocity.name.c_str());
  const int accelerometer_id = mj_name2id(model, mjOBJ_SENSOR, sensor_data.linear_acceleration.name.c_str());

  RCLCPP_ERROR_EXPRESSION(logger, quaternion_id == -1, "Failed to find IMU sensor '%s' in MuJoCo model",
                          sensor_data.orientation.name.c_str());
  RCLCPP_ERROR_EXPRESSION(logger, gyroscope_id == -1, "Failed to find IMU sensor '%s' in MuJoCo model",
                          sensor_data.angular_velocity.name.c_str());
  RCLCPP_ERROR_EXPRESSION(logger, accelerometer_id == -1, "Failed to find IMU sensor '%s' in MuJoCo model",
                          sensor_data.linear_acceleration.name.c_str());

  if (quaternion_id == -1 || gyroscope_id == -1 || accelerometer_id == -1)
  {
    return std::nullopt;
  }

  sensor_data.orientation.mj_sensor_index = model->sensor_adr[quaternion_id];
  sensor_data.angular_velocity.mj_sensor_index = model->sensor_adr[gyroscope_id];
  sensor_data.linear_acceleration.mj_sensor_index = model->sensor_adr[accelerometer_id];
  return sensor_data;
}

}  // namespace mujoco_ros2_control
