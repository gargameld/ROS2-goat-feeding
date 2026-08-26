/**
 * Copyright (c) 2025, United States Government, as represented by the
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

#include "mujoco_ros2_control/simulation/mujoco_simulation.hpp"

#include "array_safety.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control/simulation/headless_adapter.hpp"
#include "mujoco_ros2_control/simulation/mujoco_extension_loader.hpp"
#include "mujoco_ros2_control/simulation/mujoco_model_loader.hpp"

namespace mujoco_ros2_control
{
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

MujocoSimulation::~MujocoSimulation()
{
  shutdown();

  // Cleanup data and the model, if they haven't been
  if (mj_data_)
  {
    mj_deleteData(mj_data_);
  }
  if (mj_data_control_)
  {
    mj_deleteData(mj_data_control_);
  }
  if (mj_model_)
  {
    mj_deleteModel(mj_model_);
  }
  if (mj_spec_)
  {
    mj_deleteSpec(mj_spec_);
  }
}

bool MujocoSimulation::initialize(rclcpp::Node::SharedPtr node, const std::string& model_path)
{
  node_ = node;
  model_path_ = model_path;

  RCLCPP_INFO(get_logger(), "Running headless, without wall-clock synchronization.");

  // Load MuJoCo engine extensions (for example the lidar sensor plugin) so the
  // MJCF's <extension> declarations resolve when the model is compiled.
  RCLCPP_INFO(get_logger(), "Loading MuJoCo engine extensions...");
  load_mujoco_extensions();

  // Retain scope
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultPerturb(&pert_);

  RCLCPP_INFO(get_logger(), "Initializing simulation...");
  sim_ = std::make_unique<mj::Simulate>(std::make_unique<HeadlessAdapter>(), &cam_, &opt_, &pert_,
                                        /* is_passive = */ false);

  // We maintain a pointer to the mutex so that we can lock from here, too.
  // Is this a terrible idea? Maybe, but it lets us use their libraries as is...
  sim_mutex_ = &sim_->mtx;

  // Simulation time is published from the physics thread, once per step.
  RCLCPP_INFO(get_logger(), "Constructing clock publisher.");
  clock_publisher_.initialize(node_);

  // Finish initialization by loading the model and initializing the model and control data containers.
  RCLCPP_INFO(get_logger(), "Loading model...");
  const LoadedModel loaded = load_model_from_file(model_path_, get_logger());
  mj_model_ = loaded.model;
  mj_spec_ = loaded.spec;
  if (!mj_model_)
  {
    return false;
  }

  // A model MuJoCo is unhappy about starts paused so the warning can be acted on.
  if (loaded.compiled_with_warning)
  {
    sim_->run = 0;
  }

  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    mj_data_ = mj_makeData(mj_model_);
    mj_data_control_ = mj_makeData(mj_model_);
  }
  if (!mj_data_ || !mj_data_control_)
  {
    RCLCPP_FATAL(get_logger(), "Could not allocate mjData for '%s'", model_path_.c_str());
    return false;
  }

  return true;
}

void MujocoSimulation::start_physics_thread(PhysicsLoopSynchronizer* synchronizer)
{
  if (!synchronizer)
  {
    throw std::invalid_argument("A physics-loop synchronizer is required");
  }

  // When the interface is activated, we start the physics engine.
  physics_thread_ = std::thread([this, synchronizer]() {
    // Load the simulation and do an initial forward pass
    RCLCPP_INFO(get_logger(), "Starting the MuJoCo physics thread...");
    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
      sim_->m_ = mj_model_;
      sim_->d_ = mj_data_;
      mju::strcpy_arr(sim_->filename, model_path_.c_str());
    }
    // lock the sim mutex
    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
      mj_forward(mj_model_, mj_data_);
    }
    // Blocks until terminated
    physics_loop(*synchronizer);
  });
}

void MujocoSimulation::sync_control_data()
{
  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
  mj_copyData(mj_data_control_, mj_model_, mj_data_);
}

mjtNum MujocoSimulation::simulation_time() const
{
  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
  return mj_data_ ? mj_data_->time : 0.0;
}

bool MujocoSimulation::exit_requested() const
{
  return !sim_ || sim_->exitrequest.load();
}

void MujocoSimulation::shutdown()
{
  // If sim_ is created and running, clean shut it down
  if (sim_)
  {
    sim_->exitrequest.store(true);
    sim_->run = false;

    if (physics_thread_.joinable())
    {
      physics_thread_.join();
    }
  }

}

}  // namespace mujoco_ros2_control
