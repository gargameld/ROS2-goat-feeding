/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/simulation/mujoco_simulation_clock.hpp"

#include <cmath>
#include <cstdint>

#include <chrono>
#include <thread>

#include "mujoco_ros2_control/simulation/mujoco_simulation.hpp"

namespace mujoco_ros2_control
{

MujocoSimulationClock::MujocoSimulationClock(const MujocoSimulation& simulation)
  : simulation_(simulation)
{
}

mjtNum MujocoSimulationClock::get_sim_time() const
{
  return simulation_.simulation_time();
}

bool MujocoSimulationClock::sleep(mjtNum duration, const std::atomic_bool& keep_sleeping) const
{
  return sleep_until(get_sim_time() + duration, keep_sleeping);
}

bool MujocoSimulationClock::sleep_until(mjtNum wake_time, const std::atomic_bool& keep_sleeping) const
{
  while (keep_sleeping && !simulation_.exit_requested() && get_sim_time() < wake_time)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return keep_sleeping && !simulation_.exit_requested();
}

void MujocoSimulationClock::set_last_camera_update_time(mjtNum simulation_time) const
{
  const std::lock_guard<std::mutex> lock(camera_update_time_mutex_);
  last_camera_update_time_ = simulation_time;
}

std::optional<mjtNum> MujocoSimulationClock::get_last_camera_update_time() const
{
  const std::lock_guard<std::mutex> lock(camera_update_time_mutex_);
  return last_camera_update_time_;
}

void SimulationClockPublisher::initialize(const rclcpp::Node::SharedPtr& node)
{
  publisher_ = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>>(publisher_);
}

void SimulationClockPublisher::publish(mjtNum simulation_time)
{
  if (!realtime_publisher_)
  {
    return;
  }

  const auto seconds = static_cast<int32_t>(std::floor(simulation_time));
  const auto nanoseconds = static_cast<uint32_t>((simulation_time - seconds) * 1e9);

  rosgraph_msgs::msg::Clock message;
  message.clock = rclcpp::Time(seconds, nanoseconds, RCL_ROS_TIME);
  realtime_publisher_->try_publish(message);
}

}  // namespace mujoco_ros2_control
