/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <atomic>

#include <mujoco/mujoco.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace mujoco_ros2_control
{

class MujocoSimulation;

/**
 * @brief Provides simulation-time reads and waits for simulation consumers.
 */
class MujocoSimulationClock
{
public:
  /** @brief Constructs a clock backed by the supplied MuJoCo simulation. */
  explicit MujocoSimulationClock(const MujocoSimulation& simulation);

  /** @brief Returns the current MuJoCo simulation time in seconds. */
  mjtNum get_sim_time() const;

  /**
   * @brief Waits in simulation time while @p keep_sleeping remains true.
   * @return True if the full duration elapsed; false if interrupted.
   */
  bool sleep(mjtNum duration, const std::atomic_bool& keep_sleeping) const;

private:
  const MujocoSimulation& simulation_;
};

/**
 * @brief Publishes MuJoCo's simulation time to `/clock`.
 *
 * Everything downstream runs with `use_sim_time`, so this is what drives the rest of the system:
 * the physics loop publishes one timestamp per step, after the state that timestamp describes has
 * been committed for controllers to read.
 */
class SimulationClockPublisher
{
public:
  /** @brief Create the `/clock` publisher on the given node. */
  void initialize(const rclcpp::Node::SharedPtr& node);

  /**
   * @brief Publish one simulation timestamp, in seconds.
   * @note Called from the physics thread, so publishing is real-time safe and may be dropped
   *       rather than block.
   */
  void publish(mjtNum simulation_time);

private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>::SharedPtr realtime_publisher_;
};

}  // namespace mujoco_ros2_control
