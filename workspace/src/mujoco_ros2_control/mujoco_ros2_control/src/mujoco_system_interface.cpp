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

#include "mujoco_ros2_control/mujoco_system_interface.hpp"
#include "mujoco_ros2_control/system_interface/command_mode_switching.hpp"
#include "mujoco_ros2_control/system_interface/initial_state.hpp"
#include "mujoco_ros2_control/system_interface/interface_export.hpp"
#include "mujoco_ros2_control/system_interface/joint_actuator_mapping.hpp"
#include "mujoco_ros2_control/system_interface/joint_command_setup.hpp"
#include "mujoco_ros2_control/system_interface/mujoco_actuator_discovery.hpp"
#include "mujoco_ros2_control/system_interface/mujoco_model_validation.hpp"
#include "mujoco_ros2_control/system_interface/sensor_registration.hpp"
#include "mujoco_ros2_control/system_interface/simulation_configuration.hpp"
#include "mujoco_ros2_control/system_interface/state_reading.hpp"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{
MujocoSystemInterface::MujocoSystemInterface() = default;

MujocoSystemInterface::~MujocoSystemInterface()
{
  // Stop sensor threads that hold model and data pointers BEFORE the simulation is torn down.
  if (cameras_)
  {
    cameras_->close();
  }
  plugin_loader_.cleanup();

  // Stop the executor
  if (executor_)
  {
    executor_->cancel();
  }
  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }

  // Tear down the actual simulation
  simulation_.reset();

  // The synchronizer may now stop its updater thread. This must happen before
  // the shared write timestamp and mutex are destroyed.
  physics_loop_synchronizer_.reset();
}

hardware_interface::CallbackReturn
MujocoSystemInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto simulation_configuration =
      load_simulation_configuration(get_hardware_info(), get_logger());
  if (!simulation_configuration.has_value())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto node_options = make_mujoco_node_options();

  // Construct and start the ROS node spinning
  RCLCPP_INFO(get_logger(), "Constructing node and executor...");
  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  mujoco_node_ = std::make_shared<rclcpp::Node>("mujoco_ros2_control_node", node_options);
  executor_->add_node(mujoco_node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });
  RCLCPP_INFO(get_logger(), "Executor thread started.");

  // Construct the simulation wrapper with the loaded parameters.
  simulation_ = std::make_unique<MujocoSimulation>();
  if (!simulation_->initialize(get_node(), simulation_configuration->model_path))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validate_mujoco_joint_names(simulation_->model(), get_logger()))
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Register all MuJoCo actuators
  RCLCPP_INFO(get_logger(), "Registering actuators.");
  if (!discover_mujoco_actuators(simulation_->model(), mujoco_actuator_data_, get_logger()))
  {
    RCLCPP_FATAL(get_logger(), "Failed to register MuJoCo actuators, exiting...");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Pull joint and sensor information
  RCLCPP_INFO(get_logger(), "Registering joints and sensors.");
  register_urdf_joints(get_hardware_info(), simulation_->model(), mujoco_actuator_data_, urdf_joint_data_,
                       joint_hw_info_, get_logger());
  register_sensors(get_hardware_info(), simulation_->model(), sensors_hw_info_, imu_sensor_data_, get_logger());

  // Seed the simulation with the URDF's initial joint values, then publish that state to the
  // controller-facing data so the first read() observes it.
  apply_initial_joint_commands(urdf_joint_data_, mujoco_actuator_data_);
  apply_initial_pose(mujoco_actuator_data_, simulation_->data(), get_logger());
  simulation_->sync_control_data();

  // Ready cameras
  RCLCPP_INFO(get_logger(), "Initializing cameras...");
  cameras_ = std::make_unique<MujocoCameras>(get_node(), *simulation_, &simulation_->mutex(), simulation_->data(),
                                             simulation_->model(), simulation_configuration->camera_publish_rate);
  cameras_->register_cameras(get_hardware_info());

  // Verify the update rate
  const mjtNum desired_timestep = 1.0 / static_cast<double>(get_hardware_info().rw_rate);
  const bool under_sampled = simulation_->model()->opt.timestep > desired_timestep;
  RCLCPP_WARN_EXPRESSION(
      get_logger(), under_sampled,
      "MuJoCo simulator frequency %lu Hz (timestep %.6f sec) is smaller than the controller manager's update rate %lu "
      "Hz. The simulation may be under-sampled and this means that there will be some discrepancies in the rate at "
      "which controllers update cycles run. Either increase the MuJoCo timestep or decrease the controller manager's "
      "update rate.",
      static_cast<unsigned long>(1.0 / simulation_->model()->opt.timestep), simulation_->model()->opt.timestep,
      static_cast<unsigned long>(get_hardware_info().rw_rate));

  plugin_loader_.load(get_node(), simulation_->model(), simulation_->data(), simulation_->spec(),
                      &simulation_->mutex(), get_logger());

  // Start physics only after plugins are initialized so capture plugins can
  // observe the simulation from its initial state.
  physics_loop_synchronizer_ = std::make_unique<PhysicsLoopSynchronizer>(
      simulation_.get(), &last_ros_write_time_, &last_ros_write_time_mutex_, get_hardware_info());
  simulation_->start_physics_thread(physics_loop_synchronizer_.get());

  RCLCPP_INFO(get_logger(), "on_init complete.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> new_state_interfaces;
  append_joint_state_interfaces(new_state_interfaces, urdf_joint_data_, joint_hw_info_);
  append_imu_state_interfaces(new_state_interfaces, imu_sensor_data_, sensors_hw_info_);

  return new_state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> new_command_interfaces;
  append_joint_command_interfaces(new_command_interfaces, urdf_joint_data_, joint_hw_info_);

  return new_command_interfaces;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating MuJoCo hardware interface and starting Simulate threads...");

  // Start camera and sensor rendering loops
  cameras_->init();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocoSystemInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating MuJoCo hardware interface and shutting down Simulate...");

  // TODO: Should we shut MuJoCo things down here or in the destructor?

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MujocoSystemInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                   const std::vector<std::string>& stop_interfaces)
{
  // Disable stopped interfaces
  for (const auto& interface : stop_interfaces)
  {
    update_joint_control_mode(interface, false, urdf_joint_data_, mujoco_actuator_data_, get_logger());
  }

  // Enable started interfaces
  for (const auto& interface : start_interfaces)
  {
    update_joint_control_mode(interface, true, urdf_joint_data_, mujoco_actuator_data_, get_logger());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::read(const rclcpp::Time& /*time*/,
                                                            const rclcpp::Duration& /*period*/)
{
  // Joint states
  read_actuator_states(simulation_->control_data(), mujoco_actuator_data_);

  copy_actuator_states_to_joints(mujoco_actuator_data_, urdf_joint_data_);

  read_imu_states(simulation_->control_data(), imu_sensor_data_);

  // Update plugins.
  // Zero xfrc_applied first so plugins write fresh forces each control cycle (no undo needed).
  // After all updates, snapshot the result into xfrc_plugin_desired_ — the physics loop reads
  // from there so mj_copyData's viewer-force contamination never reaches the plugin buffer.
  // TODO: Break this apart when mujoco data is separated
  mju_zero(simulation_->control_data()->xfrc_applied, 6 * simulation_->model()->nbody);
  plugin_loader_.update_all(simulation_->model(), simulation_->control_data());
  mju_copy(simulation_->xfrc_plugin_desired().data(), simulation_->control_data()->xfrc_applied,
           6 * simulation_->model()->nbody);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  copy_joint_commands_to_actuators(urdf_joint_data_, mujoco_actuator_data_);

  // Joint commands
  // TODO: Support command limits. For now those ranges can be limited in the MuJoCo actuators themselves.
  for (auto& actuator : mujoco_actuator_data_)
  {
    if (actuator.actuator_type == ActuatorType::PASSIVE)
    {
      continue;
    }
    if (actuator.is_position_control_enabled)
    {
      simulation_->control_data()->ctrl[actuator.mj_actuator_id] = actuator.position_interface.command_;
    }
    else if (actuator.is_velocity_control_enabled)
    {
      simulation_->control_data()->ctrl[actuator.mj_actuator_id] = actuator.velocity_interface.command_;
    }
    else if (actuator.is_effort_control_enabled)
    {
      simulation_->control_data()->ctrl[actuator.mj_actuator_id] = actuator.effort_interface.command_;
    }
  }

  // Publish the current simulation timestamp only after this write's command data is ready.
  const auto simulation_time = rclcpp::Time(
      static_cast<int64_t>(simulation_->simulation_time() * 1'000'000'000), RCL_ROS_TIME);
  {
    const std::lock_guard<std::mutex> write_time_lock(last_ros_write_time_mutex_);
    last_ros_write_time_ = simulation_time;
  }

  return hardware_interface::return_type::OK;
}

rclcpp::Logger MujocoSystemInterface::get_logger() const
{
  return logger_;
}

rclcpp::Node::SharedPtr MujocoSystemInterface::get_node() const
{
  return mujoco_node_;
}

}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystemInterface, hardware_interface::SystemInterface);
