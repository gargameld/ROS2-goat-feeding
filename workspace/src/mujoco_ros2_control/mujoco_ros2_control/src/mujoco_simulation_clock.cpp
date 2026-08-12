/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/mujoco_simulation_clock.hpp"

#include <chrono>
#include <thread>

#include "mujoco_ros2_control/mujoco_simulation.hpp"

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

void MujocoSimulationClock::sleep(mjtNum duration) const
{
  static const std::atomic_bool always_sleep{ true };
  sleep(duration, always_sleep);
}

bool MujocoSimulationClock::sleep(mjtNum duration, const std::atomic_bool& keep_sleeping) const
{
  const mjtNum wake_time = get_sim_time() + duration;
  while (keep_sleeping && !simulation_.exit_requested() && get_sim_time() < wake_time)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return keep_sleeping && !simulation_.exit_requested();
}

}  // namespace mujoco_ros2_control
