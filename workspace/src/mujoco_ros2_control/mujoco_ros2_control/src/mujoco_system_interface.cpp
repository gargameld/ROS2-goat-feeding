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
#include "mujoco_ros2_control/detail/configuration_helpers.hpp"
#include "mujoco_ros2_control/detail/control_mode_helpers.hpp"
#include "mujoco_ros2_control/detail/initial_state_helpers.hpp"
#include "mujoco_ros2_control/detail/interface_helpers.hpp"
#include "mujoco_ros2_control/detail/model_mapping_helpers.hpp"
#include "mujoco_ros2_control/detail/plugin_helpers.hpp"
#include "mujoco_ros2_control/detail/registration_helpers.hpp"
#include "mujoco_ros2_control/detail/state_helpers.hpp"

#include <fmt/compile.h>
#include <fmt/ranges.h>

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
  if (lidar_sensors_)
  {
    lidar_sensors_->close();
  }

  // Stop plugins
  for (auto& plugin : plugin_instances_)
  {
    if (plugin)
    {
      plugin->cleanup();
    }
  }
  plugin_instances_.clear();
  transmission_instances_.clear();

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
// after humble switches from HardwareInfo to HardwareComponentInterfaceParams. This keeps it backwards compatible
// between the two distros
#if ROS_DISTRO_HUMBLE
MujocoSystemInterface::on_init(const hardware_interface::HardwareInfo& params)
#else
MujocoSystemInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
#endif
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto simulation_configuration =
      detail::load_simulation_configuration(get_hardware_info(), get_logger());
  if (!simulation_configuration.has_value())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto node_options = detail::make_mujoco_node_options(simulation_configuration.value(), get_logger());
  if (!node_options.has_value())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Construct and start the ROS node spinning
  RCLCPP_INFO(get_logger(), "Constructing node and executor...");
  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  mujoco_node_ = std::make_shared<rclcpp::Node>("mujoco_ros2_control_node", node_options.value());
  executor_->add_node(mujoco_node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });
  RCLCPP_INFO(get_logger(), "Executor thread started.");

  // Construct the simulation wrapper with the loaded parameters.
  simulation_ = std::make_unique<MujocoSimulation>();
  if (!simulation_->initialize(get_node(), simulation_configuration->model_path, simulation_configuration->model_topic,
                               simulation_configuration->speed_factor, simulation_configuration->headless))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Time publisher will be pushed from the simulation wrapper.
  RCLCPP_INFO(get_logger(), "Constructing publishers.");
  actuator_state_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>("/mujoco_actuators_states", 100);
  actuator_state_realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(actuator_state_publisher_);

  if (!detail::validate_mujoco_joint_names(simulation_->model(), get_logger()))
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Register all MuJoCo actuators
  RCLCPP_INFO(get_logger(), "Registering actuators.");
  if (!register_mujoco_actuators())
  {
    RCLCPP_FATAL(get_logger(), "Failed to register MuJoCo actuators, exiting...");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Check for free joint
  const std::string odom_free_joint_name =
      detail::get_hardware_parameter_or(get_hardware_info(), "odom_free_joint_name", "floating_base_joint");
  const auto free_joint = detail::find_free_joint(simulation_->model(), odom_free_joint_name, get_logger());
  if (!free_joint.valid)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  free_joint_id_ = free_joint.joint_id;
  free_joint_qpos_adr_ = free_joint.qpos_address;
  free_joint_qvel_adr_ = free_joint.qvel_address;

  if (free_joint_id_ != -1)
  {
    // Odometry publisher
    std::string odom_topic_name =
        detail::get_hardware_parameter_or(get_hardware_info(), "odom_topic", "/simulator/floating_base_state");
    floating_base_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);
    floating_base_realtime_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(floating_base_publisher_);

    floating_base_msg_.header.frame_id = "odom";  // TODO: Make configurable
    // Set child frame as the root link of the robot as the body attached to the free joint
    floating_base_msg_.child_frame_id = std::string(
        mj_id2name(simulation_->model(), mjtObj::mjOBJ_BODY, simulation_->model()->jnt_bodyid[free_joint_id_]));

    RCLCPP_INFO(
        get_logger(),
        "Publishing floating base odometry using the free joint : '%s' attached to the body '%s' on topic: '%s'",
        mj_id2name(simulation_->model(), mjtObj::mjOBJ_JOINT, free_joint_id_),
        floating_base_msg_.child_frame_id.c_str(), odom_topic_name.c_str());
  }

  // Pull joint and sensor information
  RCLCPP_INFO(get_logger(), "Registering joints and sensors.");
  register_urdf_joints(get_hardware_info());
  register_sensors(get_hardware_info());
  if (!register_transmissions(get_hardware_info()))
  {
    RCLCPP_FATAL(get_logger(), "Failed to register transmissions, exiting...");
    return hardware_interface::CallbackReturn::FAILURE;
  }
  initialize_initial_positions(get_hardware_info());
  set_initial_pose();

  // Store initial state for reset_world service
  simulation_->capture_initial_state();

  // This CB will be triggered by the MujocoSimulation after resettting the sim and qpos/qvel/ctrl have been restored.
  simulation_->set_reset_callback([this](bool fill_initial_state) { this->reset_simulation_state(fill_initial_state); });

  // Ready cameras
  RCLCPP_INFO(get_logger(), "Initializing cameras...");
  cameras_ = std::make_unique<MujocoCameras>(get_node(), *simulation_, &simulation_->mutex(), simulation_->data(),
                                             simulation_->model(), simulation_configuration->camera_publish_rate);
  cameras_->register_cameras(get_hardware_info());

  // Configure Lidar sensors
  RCLCPP_INFO(get_logger(), "Initializing lidar...");
  lidar_sensors_ = std::make_unique<MujocoLidar>(get_node(), &simulation_->mutex(), simulation_->data(),
                                                 simulation_->model(), simulation_configuration->lidar_publish_rate);
  if (!lidar_sensors_->register_lidar(get_hardware_info()))
  {
    RCLCPP_INFO(get_logger(), "Failed to initialize lidar, exiting...");
    return hardware_interface::CallbackReturn::FAILURE;
  }

#if !ROS_DISTRO_HUMBLE
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
#endif

  actuator_state_msg_.name.clear();
  for (const auto& actuator : mujoco_actuator_data_)
  {
    actuator_state_msg_.name.push_back(actuator.joint_name);
  }

  // Load MuJoCo ROS2 Control plugins
  this->load_mujoco_plugins();

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
  detail::append_joint_state_interfaces(new_state_interfaces, urdf_joint_data_, joint_hw_info_);
  detail::append_force_torque_state_interfaces(new_state_interfaces, ft_sensor_data_, sensors_hw_info_);
  detail::append_imu_state_interfaces(new_state_interfaces, imu_sensor_data_, sensors_hw_info_);

  return new_state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> new_command_interfaces;
  detail::append_joint_command_interfaces(new_command_interfaces, urdf_joint_data_, joint_hw_info_);

  return new_command_interfaces;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating MuJoCo hardware interface and starting Simulate threads...");

  // Start camera and sensor rendering loops
  cameras_->init();
  lidar_sensors_->init();

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
    detail::update_joint_control_mode(interface, false, get_hardware_info(), simulation_->model(), urdf_joint_data_,
                                      mujoco_actuator_data_, get_logger());
  }

  // Enable started interfaces
  for (const auto& interface : start_interfaces)
  {
    detail::update_joint_control_mode(interface, true, get_hardware_info(), simulation_->model(), urdf_joint_data_,
                                      mujoco_actuator_data_, get_logger());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::read(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  // Joint states
  actuator_state_msg_.header.stamp = time;
  detail::read_actuator_states(simulation_->control_data(), mujoco_actuator_data_, actuator_state_msg_);
  // Publish actuator states
  if (actuator_state_realtime_publisher_)
  {
#if ROS_DISTRO_HUMBLE
    actuator_state_realtime_publisher_->tryPublish(actuator_state_msg_);
#else
    actuator_state_realtime_publisher_->try_publish(actuator_state_msg_);
#endif
  }

  actuator_state_to_joint_state();

  detail::read_imu_states(simulation_->control_data(), imu_sensor_data_);
  detail::read_force_torque_states(simulation_->control_data(), ft_sensor_data_);

  // Publish Odometry
  if (free_joint_id_ != -1 && floating_base_realtime_publisher_)
  {
    floating_base_msg_.header.stamp = time;
    detail::populate_floating_base_odometry(simulation_->control_data(), free_joint_qpos_adr_, free_joint_qvel_adr_,
                                            floating_base_msg_);

#if ROS_DISTRO_HUMBLE
    floating_base_realtime_publisher_->tryPublish(floating_base_msg_);
#else
    floating_base_realtime_publisher_->try_publish(floating_base_msg_);
#endif
  }

  // Update plugins.
  // Zero xfrc_applied first so plugins write fresh forces each control cycle (no undo needed).
  // After all updates, snapshot the result into xfrc_plugin_desired_ — the physics loop reads
  // from there so mj_copyData's viewer-force contamination never reaches the plugin buffer.
  // TODO: Break this apart when mujoco data is separated
  mju_zero(simulation_->control_data()->xfrc_applied, 6 * simulation_->model()->nbody);
  for (auto& plugin : plugin_instances_)
  {
    plugin->update(simulation_->model(), simulation_->control_data());
  }
  mju_copy(simulation_->xfrc_plugin_desired().data(), simulation_->control_data()->xfrc_applied,
           6 * simulation_->model()->nbody);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::write(const rclcpp::Time&,
                                                             const rclcpp::Duration& period)
{
  detail::update_mimic_joint_commands(urdf_joint_data_);

  joint_command_to_actuator_command();

  // portable lambda function to compute pid command using either function name for the correct distro
  auto pid_compute_command = [](auto& pid, const auto& error, const auto& period_t) -> double {
#if ROS_DISTRO_HUMBLE
    return pid->computeCommand(error, period_t);
#else
    return pid->compute_command(error, period_t);
#endif
  };

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
    else if (actuator.is_position_pid_control_enabled)
    {
      const double error = actuator.position_interface.command_ - simulation_->data()->qpos[actuator.mj_pos_adr];
      simulation_->control_data()->ctrl[actuator.mj_actuator_id] = pid_compute_command(actuator.pos_pid, error, period);
    }
    else if (actuator.is_velocity_control_enabled)
    {
      simulation_->control_data()->ctrl[actuator.mj_actuator_id] = actuator.velocity_interface.command_;
    }
    else if (actuator.is_velocity_pid_control_enabled)
    {
      const double error = actuator.velocity_interface.command_ - simulation_->data()->qvel[actuator.mj_vel_adr];
      simulation_->control_data()->ctrl[actuator.mj_actuator_id] = pid_compute_command(actuator.vel_pid, error, period);
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

void MujocoSystemInterface::actuator_state_to_joint_state()
{
  // actuator: MuJoCo -> transmission
  std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(),
                [](auto& actuator_interface) { actuator_interface.copy_state_to_transmission(); });

  // transmission: actuator -> joint
  std::for_each(transmission_instances_.begin(), transmission_instances_.end(),
                [](auto& transmission) { transmission->actuator_to_joint(); });

  // joint: transmission -> state
  std::for_each(urdf_joint_data_.begin(), urdf_joint_data_.end(),
                [](auto& joint_interface) { joint_interface.copy_state_from_transmission(); });

  // If the actuator name and joint name is same (which is the case for non transmission joints), we need to copy
  // the state from actuator to joint here as there is no transmission instance to do that.
  for (auto& joint : urdf_joint_data_)
  {
    std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(), [&](auto& actuator_interface) {
      if (actuator_interface.joint_name == joint.name)
      {
        joint.position_interface.state_ = actuator_interface.position_interface.state_;
        joint.velocity_interface.state_ = actuator_interface.velocity_interface.state_;
        joint.effort_interface.state_ = actuator_interface.effort_interface.state_;
      }
    });
  }
}

void MujocoSystemInterface::joint_command_to_actuator_command()
{
  // Transmissions
  std::for_each(urdf_joint_data_.begin(), urdf_joint_data_.end(),
                [](auto& joint_interface) { joint_interface.copy_command_to_transmission(); });

  // transmission -> actuator
  std::for_each(transmission_instances_.begin(), transmission_instances_.end(),
                [](auto& transmission) { transmission->joint_to_actuator(); });

  // set the commands to the MuJoCo actuators
  std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(),
                [](auto& actuator_interface) { actuator_interface.copy_command_from_transmission(); });

  // If the actuator name and joint name is same (which is the case for non transmission joints), we need to copy
  // the command from joint to actuator here as there is no transmission instance to do that.
  for (auto& joint : urdf_joint_data_)
  {
    std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(), [&](auto& actuator_interface) {
      if (actuator_interface.joint_name == joint.name && actuator_interface.actuator_type != ActuatorType::PASSIVE)
      {
        actuator_interface.position_interface.command_ = joint.position_interface.command_;
        actuator_interface.velocity_interface.command_ = joint.velocity_interface.command_;
        actuator_interface.effort_interface.command_ = joint.effort_interface.command_;
      }
    });
  }
}

bool MujocoSystemInterface::register_mujoco_actuators()
{
  mujoco_actuator_data_.clear();
  mujoco_actuator_data_.resize(simulation_->model()->nu);

  // Pull the name of the file to load for starting config, if present. We only override start position if that
  // parameter exists and it is not an empty string
  override_mujoco_actuator_positions_ = false;
  auto it = get_hardware_info().hardware_parameters.find("override_start_position_file");
  if (it != get_hardware_info().hardware_parameters.end())
  {
    override_mujoco_actuator_positions_ = !it->second.empty();
  }

  // If we have that file present, load the initial positions from that file to the appropriate simulation_->data() structures
  if (override_mujoco_actuator_positions_)
  {
    std::string override_start_position_file = it->second;
    bool success = set_override_start_positions(override_start_position_file);
    if (!success)
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to load override start positions from %s. Falling back to urdf initial positions.",
                   override_start_position_file.c_str());
      override_mujoco_actuator_positions_ = false;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Loaded initial positions from file %s.", override_start_position_file.c_str());
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "override_start_position_file not passed. Loading initial positions from ros2_control xacro.");
  }

  for (int i = 0; i < simulation_->model()->nu; i++)
  {
    RCLCPP_DEBUG(get_logger(), "Registering MuJoCo actuator %ld/%ld", static_cast<long>(i + 1),
                 static_cast<long>(simulation_->model()->nu));
    MuJoCoActuatorData& actuator_data = mujoco_actuator_data_.at(i);
    if (!detail::populate_actuator_model_data(simulation_->model(), i, actuator_data, get_logger()))
    {
      return false;
    }
    detail::initialize_actuator_control(actuator_data, get_node());

    const char* act_name = mj_id2name(simulation_->model(), mjOBJ_ACTUATOR, i);
    if (!act_name)
    {
      act_name = "unnamed";
    }
    RCLCPP_DEBUG(get_logger(), "Successfully registered actuator '%s'", act_name);
  }

  // now look out for the MuJoCo joints that do not have any actuator associated with them
  detail::append_passive_actuators(simulation_->model(), get_hardware_info(), mujoco_actuator_data_, get_logger());

  // Override initial positions with a keyframe if specified
  if (!override_mujoco_actuator_positions_)
  {
    const std::string keyframe_name =
        detail::get_hardware_parameter_or(get_hardware_info(), "initial_keyframe", "");
    if (!keyframe_name.empty())
    {
      initial_keyframe_ = keyframe_name;
      RCLCPP_INFO(get_logger(), "Applying initial keyframe: '%s'", initial_keyframe_.c_str());
      override_mujoco_actuator_positions_ = simulation_->apply_keyframe(initial_keyframe_);
      if (!override_mujoco_actuator_positions_)
      {
        RCLCPP_ERROR(get_logger(), "Failed to apply initial keyframe: '%s'", initial_keyframe_.c_str());
        return false;
      }
    }
  }

  // Set initial values if they are set in the info, or from override start position file
  if (override_mujoco_actuator_positions_)
  {
    RCLCPP_DEBUG(get_logger(),
                 "Initializing actuator position states from override start position file for %zu actuators.",
                 mujoco_actuator_data_.size());

    detail::initialize_actuator_states(simulation_->data(), mujoco_actuator_data_);
  }
  return true;
}

void MujocoSystemInterface::register_urdf_joints(const hardware_interface::HardwareInfo& hardware_info)
{
  RCLCPP_INFO(get_logger(), "Registering joints...");
  urdf_joint_data_.resize(hardware_info.joints.size());

  for (size_t joint_index = 0; joint_index < hardware_info.joints.size(); joint_index++)
  {
    auto joint = hardware_info.joints.at(joint_index);
    const std::string actuator_name =
        detail::get_joint_actuator_name(joint.name, hardware_info, simulation_->model());

    // Get the information for the URDF Joint data
    URDFJointData& joint_data = urdf_joint_data_.at(joint_index);
    joint_data.name = joint.name;

    detail::configure_mimic_joint(joint, hardware_info, joint_data, get_logger());

    auto* actuator =
        detail::find_controllable_actuator(mujoco_actuator_data_, simulation_->model(), actuator_name);
    const bool actuator_exists = actuator != nullptr;
    // This isn't a failure the joint just won't be controllable
    RCLCPP_INFO_EXPRESSION(get_logger(), !actuator_exists && !joint_data.is_mimic,
                           "Failed to find actuator for joint : %s. This joint will be treated as a passive joint.",
                           joint.name.c_str());
    RCLCPP_INFO_EXPRESSION(get_logger(), joint.command_interfaces.empty() && !joint_data.is_mimic,
                           "Joint : %s is a passive joint", joint.name.c_str());
    if (!joint.command_interfaces.empty() && !actuator_exists)
    {
      RCLCPP_ERROR(get_logger(),
                   "Joint : %s has command interfaces defined but no matching actuator in the MuJoCo model. This joint "
                   "will be treated as a passive joint and no command interfaces will be exported.",
                   joint.name.c_str());
      joint.command_interfaces.clear();
    }

    // Add to the joint hw information map
    joint_hw_info_.insert(std::make_pair(joint.name, joint));

    // Set initial values to joint interfaces if they are set in the info
    if (!override_mujoco_actuator_positions_)
    {
      override_urdf_joint_positions_ = true;
      detail::initialize_joint_interfaces(joint, joint_data);
    }

    const auto command_interface_names = detail::get_ordered_command_interfaces(joint);
    joint_data.command_interfaces = command_interface_names;

    if (actuator)
    {
      detail::configure_joint_command_interfaces(joint, actuator_name, command_interface_names, *actuator,
                                                 get_logger());
    }
  }
}

bool MujocoSystemInterface::register_transmissions(const hardware_interface::HardwareInfo& hardware_info)
{
  transmission_instances_.clear();
  auto hardware_transmissions = hardware_info.transmissions;
  transmission_loader_ = std::make_unique<pluginlib::ClassLoader<transmission_interface::TransmissionLoader>>(
      "transmission_interface", "transmission_interface::TransmissionLoader");

  for (const auto& t_info : hardware_transmissions)
  {
    if (t_info.joints.empty() || t_info.actuators.empty())
    {
      RCLCPP_FATAL(get_logger(), "Transmission '%s' has no joints or actuators defined", t_info.name.c_str());
      return false;
    }

    if (!detail::transmission_actuators_exist(t_info, simulation_->model(), get_logger()))
    {
      RCLCPP_ERROR(get_logger(),
                   "Not all transmission actuators and joints for transmission '%s' found as MuJoCo actuators. This "
                   "shouldn't happen.",
                   t_info.name.c_str());
      return false;
    }

    if (!transmission_loader_->isClassAvailable(t_info.type))
    {
      RCLCPP_FATAL(get_logger(), "Transmission '%s' of type '%s' not available", t_info.name.c_str(),
                   t_info.type.c_str());
      return false;
    }

    if (!detail::transmission_joint_interfaces_match(t_info, get_logger()))
    {
      return false;
    }

    std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
    try
    {
      auto loader = transmission_loader_->createSharedInstance(t_info.type);
      transmission = loader->load(t_info);
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(get_logger(), "Caught exception when trying to create transmission loader of type %s : %s",
                   t_info.type.c_str(), e.what());
      return false;
    }

    // Create the joint_handles vector for each joint in the transmission
    std::vector<transmission_interface::JointHandle> joint_handles;
    RCLCPP_INFO(get_logger(), "Creating joint and actuator handles for transmission: %s", t_info.name.c_str());
    if (!detail::make_transmission_joint_handles(t_info, urdf_joint_data_, joint_handles, get_logger()))
    {
      return false;
    }

    // Create the actuator_handles vector for each actuator in the transmission
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    if (!detail::make_transmission_actuator_handles(t_info, mujoco_actuator_data_, actuator_handles, get_logger()))
    {
      return false;
    }

    try
    {
      transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException& exc)
    {
      RCLCPP_FATAL(get_logger(), "Error while configuring %s: %s", t_info.name.c_str(), exc.what());
      return false;
    }

    transmission_instances_.push_back(transmission);
  }
  RCLCPP_INFO_EXPRESSION(get_logger(), !transmission_instances_.empty(), "Registered %zu transmissions",
                         transmission_instances_.size());

  return true;
}

bool MujocoSystemInterface::initialize_initial_positions(const hardware_interface::HardwareInfo& /*hardware_info*/)
{
  if (override_mujoco_actuator_positions_)
  {
    // Transforms the actuators' state to the joint state interfaces
    actuator_state_to_joint_state();

    // Set the initial joint state as joint commands
    std::for_each(urdf_joint_data_.begin(), urdf_joint_data_.end(),
                  [](auto& joint_interface) { joint_interface.copy_state_to_command(); });
  }
  if (override_urdf_joint_positions_)
  {
    // Transforms the joints' command to the actuator command interfaces
    joint_command_to_actuator_command();

    // Set the initial actuator commands as actuator states
    std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(),
                  [](auto& actuator_interface) { actuator_interface.copy_command_to_state(); });

    detail::copy_passive_joint_states(urdf_joint_data_, mujoco_actuator_data_);
  }
  return true;
}

void MujocoSystemInterface::register_sensors(const hardware_interface::HardwareInfo& hardware_info)
{
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    auto sensor = hardware_info.sensors.at(sensor_index);
    const std::string sensor_name = sensor.name;

    if (sensor.parameters.count("mujoco_type") == 0)
    {
      RCLCPP_INFO(get_logger(), "Not adding hardware interface for sensor in ros2_control xacro: '%s'",
                  sensor_name.c_str());
      continue;
    }
    const auto mujoco_type = sensor.parameters.at("mujoco_type");

    // If there is a specific sensor name provided we use that, otherwise we assume the MuJoCo model's
    // sensor is named identically to the ros2_control hardware interface's.
    std::string mujoco_sensor_name;
    if (sensor.parameters.count("mujoco_sensor_name") == 0)
    {
      mujoco_sensor_name = sensor_name;
    }
    else
    {
      mujoco_sensor_name = sensor.parameters.at("mujoco_sensor_name");
    }

    RCLCPP_INFO(get_logger(), "Adding sensor named: '%s', of type: '%s', mapping to the MJCF sensor: '%s'",
                sensor_name.c_str(), mujoco_type.c_str(), mujoco_sensor_name.c_str());

    // Add to the sensor hw information map
    sensors_hw_info_.insert(std::make_pair(sensor_name, sensor));

    if (mujoco_type == "fts")
    {
      const auto sensor_data = detail::make_force_torque_sensor(sensor, mujoco_sensor_name, get_hardware_info(),
                                                               simulation_->model(), get_logger());
      if (sensor_data.has_value())
      {
        ft_sensor_data_.push_back(sensor_data.value());
      }
    }

    else if (mujoco_type == "imu")
    {
      const auto sensor_data = detail::make_imu_sensor(sensor, mujoco_sensor_name, get_hardware_info(),
                                                      simulation_->model(), get_logger());
      if (sensor_data.has_value())
      {
        imu_sensor_data_.push_back(sensor_data.value());
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid mujoco_type passed to the MuJoCo hardware interface: '%s'",
                   mujoco_type.c_str());
    }
  }
}

bool MujocoSystemInterface::set_override_start_positions(const std::string& override_start_position_file)
{
  const auto initial_state = detail::load_initial_state_values(override_start_position_file, get_logger());
  if (!initial_state.has_value() ||
      !detail::initial_state_sizes_match(initial_state.value(), simulation_->model(), get_logger()))
  {
    return false;
  }

  detail::copy_initial_state_to_data(initial_state.value(), simulation_->data());
  return true;
}

void MujocoSystemInterface::set_initial_pose()
{
  detail::apply_initial_pose(mujoco_actuator_data_, simulation_->data(), get_logger());
  // Copy into the control data for reads
  mj_copyData(simulation_->control_data(), simulation_->model(), simulation_->data());
}

void MujocoSystemInterface::reset_simulation_state(bool /*fill_initial_state*/)
{
  /// @note This method assumes sim_mutex_ is already held by the caller

  detail::reset_actuator_interfaces(mujoco_actuator_data_, simulation_->data(), simulation_->control_data());

  // Update URDF joint states from actuator states
  actuator_state_to_joint_state();

  detail::reset_joint_commands(urdf_joint_data_);
}

void MujocoSystemInterface::get_model(mjModel*& dest)
{
  const std::unique_lock<std::recursive_mutex> lock(simulation_->mutex());
  dest = mj_copyModel(dest, simulation_->model());
}

void MujocoSystemInterface::get_data(mjData*& dest)
{
  const std::unique_lock<std::recursive_mutex> lock(simulation_->mutex());
  if (dest == nullptr)
  {
    dest = mj_makeData(simulation_->model());
  }
  mj_copyData(dest, simulation_->model(), simulation_->data());
}

void MujocoSystemInterface::set_data(mjData* mj_data)
{
  const std::unique_lock<std::recursive_mutex> lock(simulation_->mutex());
  mj_copyData(simulation_->data(), simulation_->model(), mj_data);
}

rclcpp::Logger MujocoSystemInterface::get_logger() const
{
  return logger_;
}

rclcpp::Node::SharedPtr MujocoSystemInterface::get_node() const
{
  return mujoco_node_;
}

void MujocoSystemInterface::load_mujoco_plugins()
{
  try
  {
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase>>(
        "mujoco_ros2_control_plugins", "mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase");

    // Get list of plugins from parameter (if specified)
    const std::string mujoco_plugins_param_prefix = "mujoco_plugins";
    const auto plugins_ns = detail::discover_plugin_names(get_node(), mujoco_plugins_param_prefix);
    RCLCPP_INFO_EXPRESSION(get_logger(), plugins_ns.empty(), "No 'mujoco_plugins' parameter found!");
    RCLCPP_INFO_EXPRESSION(get_logger(), !plugins_ns.empty(),
                           "Found 'mujoco_plugins' parameter with the following plugins: %s",
                           fmt::format("{}", fmt::join(plugins_ns, ", ")).c_str());

    // Load and initialize each plugin
    for (const auto& plugin_name : plugins_ns)
    {
      try
      {
        const std::string plugin_type_param = mujoco_plugins_param_prefix + "." + plugin_name + ".type";
        if (!get_node()->has_parameter(plugin_type_param))
        {
          RCLCPP_WARN(get_logger(), "Plugin parameter '%s' not found, skipping plugin.", plugin_type_param.c_str());
          continue;
        }
        const std::string plugin_type = get_node()->get_parameter(plugin_type_param).as_string();
        auto plugin = plugin_loader_->createSharedInstance(plugin_type);
        plugin->set_simulation_mutex(&simulation_->mutex());
        plugin->set_mujoco_spec(simulation_->spec());
        if (plugin->init(get_node()->create_sub_node(plugin_name), simulation_->model(), simulation_->data()))
        {
          plugin_instances_.push_back(plugin);
          RCLCPP_INFO(get_logger(), "Successfully loaded and initialized plugin: %s", plugin_name.c_str());
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "Failed to initialize plugin: %s of type: %s", plugin_name.c_str(),
                       plugin_type.c_str());
          throw std::runtime_error("Failed to initialize plugin: " + plugin_name + " of type: " + plugin_type);
        }
      }
      catch (const pluginlib::PluginlibException& ex)
      {
        RCLCPP_ERROR(get_logger(), "Failed to load plugin '%s': %s", plugin_name.c_str(), ex.what());
        throw;  // re-throw to be caught by the outer catch block
      }
    }
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create plugin loader: %s", ex.what());
  }
}

}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystemInterface, hardware_interface::SystemInterface);
