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
   * @brief Waits until @p duration seconds have elapsed in simulation time.
   *
   * Returns early when the simulation is shutting down.
   */
  void sleep(mjtNum duration) const;

  /**
   * @brief Waits in simulation time while @p keep_sleeping remains true.
   * @return True if the full duration elapsed; false if interrupted.
   */
  bool sleep(mjtNum duration, const std::atomic_bool& keep_sleeping) const;

private:
  const MujocoSimulation& simulation_;
};

}  // namespace mujoco_ros2_control
