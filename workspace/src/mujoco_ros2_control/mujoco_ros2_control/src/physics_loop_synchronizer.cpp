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

#include "mujoco_ros2_control/physics_loop_synchronizer.hpp"
#include "mujoco_ros2_control/mujoco_simulation.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <unistd.h>

namespace mujoco_ros2_control
{
namespace
{

unsigned int extract_write_rate(const hardware_interface::HardwareInfo& hardware_info,
                                const std::string& parameter_name)
{
  const auto parameter = hardware_info.hardware_parameters.find(parameter_name);
  if (parameter == hardware_info.hardware_parameters.end())
  {
    throw std::invalid_argument("Missing hardware parameter: " + parameter_name);
  }

  std::size_t parsed_characters = 0;
  const unsigned long parsed_rate = std::stoul(parameter->second, &parsed_characters);
  if (parsed_characters != parameter->second.size() || parsed_rate == 0 ||
      parsed_rate > std::numeric_limits<unsigned int>::max())
  {
    throw std::invalid_argument("Hardware parameter '" + parameter_name + "' must be a positive integer");
  }

  return static_cast<unsigned int>(parsed_rate);
}

double extract_safety_interval(const hardware_interface::HardwareInfo& hardware_info)
{
  constexpr double default_safety_interval_seconds = 0.002;
  const auto parameter = hardware_info.hardware_parameters.find("physics_sync_safety_interval");
  if (parameter == hardware_info.hardware_parameters.end())
  {
    return default_safety_interval_seconds;
  }

  std::size_t parsed_characters = 0;
  const double interval = std::stod(parameter->second, &parsed_characters);
  if (parsed_characters != parameter->second.size() || !std::isfinite(interval) || interval < 0.0)
  {
    throw std::invalid_argument("Hardware parameter 'physics_sync_safety_interval' must be non-negative");
  }
  return interval;
}

std::chrono::duration<double, std::milli> extract_extra_wait_time(
  const hardware_interface::HardwareInfo& hardware_info)
{
  constexpr auto default_extra_wait_time = std::chrono::duration<double, std::milli>{ 0.0 };
  const auto parameter = hardware_info.hardware_parameters.find("extra_wait_time");
  if (parameter == hardware_info.hardware_parameters.end())
  {
    return default_extra_wait_time;
  }

  std::size_t parsed_characters = 0;
  const double milliseconds = std::stod(parameter->second, &parsed_characters);
  if (parsed_characters != parameter->second.size() || !std::isfinite(milliseconds) || milliseconds < 0.0)
  {
    throw std::invalid_argument("Hardware parameter 'extra_wait_time' must be a non-negative number of milliseconds");
  }

  return std::chrono::duration<double, std::milli>{ milliseconds };
}

void set_current_thread_to_low_priority()
{
  constexpr int low_priority_nice_value = 10;
  errno = 0;
  const int result = setpriority(PRIO_PROCESS, static_cast<id_t>(gettid()), low_priority_nice_value);
  RCLCPP_WARN_EXPRESSION(rclcpp::get_logger("PhysicsLoopSynchronizer"), result != 0,
                         "Could not lower the expected-write updater priority: %s", std::strerror(errno));
}

}  // namespace

PhysicsLoopSynchronizer::PhysicsLoopSynchronizer(MujocoSimulation* simulation,
                                                 const rclcpp::Time* last_ros_write_time,
                                                 std::mutex* last_ros_write_time_mutex,
                                                 const hardware_interface::HardwareInfo& hardware_info)
  : simulation_(simulation),
    last_ros_write_time_(last_ros_write_time),
    last_ros_write_time_mutex_(last_ros_write_time_mutex),
    write_period_seconds_(1.0 / static_cast<double>(extract_write_rate(hardware_info, "write_frequency"))),
    safety_time_interval_seconds_(extract_safety_interval(hardware_info)),
    extra_wait_time_(extract_extra_wait_time(hardware_info))
{
  if (!simulation_ || !last_ros_write_time_ || !last_ros_write_time_mutex_)
  {
    throw std::invalid_argument("PhysicsLoopSynchronizer requires valid simulation and write-time pointers");
  }

  {
    const std::lock_guard<std::mutex> write_time_lock(*last_ros_write_time_mutex_);
    next_expected_write_time_ =
        *last_ros_write_time_ + rclcpp::Duration::from_seconds(write_period_seconds_);
  }

  expected_write_time_thread_ =
      std::thread(&PhysicsLoopSynchronizer::update_expected_write_time_loop, this);

  for (const auto& [key, value] : hardware_info.hardware_parameters)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("PhysicsLoopSynchronizer"),
      "hardware parameter: '%s' = '%s'",
      key.c_str(), value.c_str());
  }
}

PhysicsLoopSynchronizer::~PhysicsLoopSynchronizer()
{
  updater_running_.store(false);
  if (expected_write_time_thread_.joinable())
  {
    expected_write_time_thread_.join();
  }
}

void PhysicsLoopSynchronizer::sync_physics_loop() const
{
  std::this_thread::sleep_for(extra_wait_time_);

  const double current_simulation_time = simulation_->simulation_time();

  while (!simulation_->exit_requested())
  {
    double next_expected_write_time = 0.0;
    {
      const std::lock_guard<std::mutex> expected_time_lock(expected_write_time_mutex_);
      next_expected_write_time = next_expected_write_time_.seconds();
    }

    if (current_simulation_time <= next_expected_write_time + safety_time_interval_seconds_)
    {
      return;
    }

    std::this_thread::yield();
  }
}

void PhysicsLoopSynchronizer::update_expected_write_time_loop()
{
  using namespace std::chrono_literals;
  set_current_thread_to_low_priority();

  rclcpp::Time observed_write_time(0, 0, RCL_ROS_TIME);
  while (updater_running_.load())
  {
    rclcpp::Time current_write_time(0, 0, RCL_ROS_TIME);
    {
      const std::lock_guard<std::mutex> write_time_lock(*last_ros_write_time_mutex_);
      current_write_time = *last_ros_write_time_;
    }

    if (current_write_time != observed_write_time)
    {
      const auto next_write_time =
          current_write_time + rclcpp::Duration::from_seconds(write_period_seconds_);
      {
        const std::lock_guard<std::mutex> expected_time_lock(expected_write_time_mutex_);
        next_expected_write_time_ = next_write_time;
      }
      observed_write_time = current_write_time;
    }

    std::this_thread::sleep_for(1ms);
  }
}

}  // namespace mujoco_ros2_control
