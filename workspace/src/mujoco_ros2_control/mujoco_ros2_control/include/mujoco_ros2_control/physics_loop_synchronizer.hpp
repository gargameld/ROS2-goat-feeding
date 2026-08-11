/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/time.hpp>

namespace mujoco_ros2_control
{

class MujocoSimulation;

/**
 * @brief Prevents physics from advancing beyond the next expected ROS write.
 *
 * The simulation, timestamp, and timestamp mutex must outlive this object.
 */
class PhysicsLoopSynchronizer
{
public:
  PhysicsLoopSynchronizer(MujocoSimulation* simulation, const rclcpp::Time* last_ros_write_time,
                          std::mutex* last_ros_write_time_mutex,
                          const hardware_interface::HardwareInfo& hardware_info);
  ~PhysicsLoopSynchronizer();

  PhysicsLoopSynchronizer(const PhysicsLoopSynchronizer&) = delete;
  PhysicsLoopSynchronizer& operator=(const PhysicsLoopSynchronizer&) = delete;

  /**
   * @brief Yield until the next expected ROS write is no longer overdue.
   */
  void sync_physics_loop() const;

private:
  void update_expected_write_time_loop();

  MujocoSimulation* simulation_;
  const rclcpp::Time* last_ros_write_time_;
  std::mutex* last_ros_write_time_mutex_;

  const double write_period_seconds_;
  const double safety_time_interval_seconds_;
  const std::chrono::milliseconds extra_wait_time_;

  mutable std::mutex expected_write_time_mutex_;
  rclcpp::Time next_expected_write_time_{ 0, 0, RCL_ROS_TIME };

  std::atomic<bool> updater_running_{ true };
  std::thread expected_write_time_thread_;
};

}  // namespace mujoco_ros2_control
