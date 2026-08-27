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

#include "mujoco_ros2_control/simulation/physics_loop_synchronizer.hpp"
#include "mujoco_ros2_control/hardware_parameters.hpp"
#include "mujoco_ros2_control/simulation/mujoco_simulation.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <unistd.h>

namespace mujoco_ros2_control
{
namespace
{

constexpr double default_safety_interval_seconds = 0.002;
constexpr double default_max_camera_lag_seconds = 1.0;
constexpr double default_extra_wait_time_milliseconds = 0.0;

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
  : PhysicsLoopSynchronizer(simulation, last_ros_write_time, last_ros_write_time_mutex,
                            HardwareParameters(hardware_info))
{
}

PhysicsLoopSynchronizer::PhysicsLoopSynchronizer(MujocoSimulation* simulation,
                                                 const rclcpp::Time* last_ros_write_time,
                                                 std::mutex* last_ros_write_time_mutex,
                                                 const HardwareParameters& parameters)
  : simulation_(simulation),
    last_ros_write_time_(last_ros_write_time),
    last_ros_write_time_mutex_(last_ros_write_time_mutex),
    write_period_seconds_(1.0 / static_cast<double>(parameters.get_positive_unsigned("write_frequency"))),
    safety_time_interval_seconds_(
        parameters.get_non_negative_double("physics_sync_safety_interval", default_safety_interval_seconds)),
    max_camera_lag_seconds_(parameters.get_non_negative_double("max_camera_lag", default_max_camera_lag_seconds)),
    extra_wait_time_(
        parameters.get_non_negative_double("extra_wait_time", default_extra_wait_time_milliseconds)),
    required_controller_names_(parameters.get_string_list("required_active_controllers"))
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

  initialize_physics_sync_node();

  expected_write_time_thread_ =
      std::thread(&PhysicsLoopSynchronizer::update_expected_write_time_loop, this);

  parameters.log_all(rclcpp::get_logger("PhysicsLoopSynchronizer"));
}

PhysicsLoopSynchronizer::~PhysicsLoopSynchronizer()
{
  updater_running_.store(false);
  physics_sync_executor_->cancel();
  if (expected_write_time_thread_.joinable())
  {
    expected_write_time_thread_.join();
  }
  if (physics_sync_executor_thread_.joinable())
  {
    physics_sync_executor_thread_.join();
  }
}

void PhysicsLoopSynchronizer::sync_physics_loop() const
{
  while (simulation_paused_.load(std::memory_order_acquire) && !simulation_->exit_requested())
  {
    std::this_thread::yield();
  }

  if (simulation_->exit_requested())
  {
    return;
  }

  // Let the first physics step publish a non-zero /clock value. The controller
  // manager needs that initial tick to leave wait_until_started() and process
  // controller activation requests. All subsequent steps wait for activation.
  if (!initial_sync_completed_.exchange(true, std::memory_order_acq_rel))
  {
    return;
  }

  while (!all_controllers_are_active() && !simulation_->exit_requested())
  {
    std::this_thread::yield();
  }

  if (simulation_->exit_requested())
  {
    return;
  }

  if (!controller_activation_logged_.exchange(true, std::memory_order_acq_rel))
  {
    RCLCPP_INFO(
      rclcpp::get_logger("PhysicsLoopSynchronizer"),
      "All required controllers are active; releasing the physics loop.");
  }

  std::this_thread::sleep_for(extra_wait_time_);

  wait_for_cameras_to_catch_up();

  if (simulation_->exit_requested())
  {
    return;
  }

  const double current_simulation_time = simulation_->simulation_time();

  while (!simulation_->exit_requested())
  {
    if (simulation_paused_.load(std::memory_order_acquire))
    {
      std::this_thread::yield();
      continue;
    }

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

void PhysicsLoopSynchronizer::wait_for_cameras_to_catch_up() const
{
  // Rendering is far slower than physics, so the cameras fall behind unless the simulation
  // waits for them. Nothing happens until the first point cloud has been produced: if camera
  // rendering never starts, the physics loop must not stall on it.
  while (!simulation_->exit_requested())
  {
    const auto last_camera_update_time = simulation_->clock().get_last_camera_update_time();
    if (!last_camera_update_time.has_value() ||
        simulation_->simulation_time() - *last_camera_update_time <= max_camera_lag_seconds_)
    {
      return;
    }

    // Polling instead of yielding: simulation_time() takes the simulation mutex, and the
    // camera thread needs that same mutex to grab its next snapshot.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool PhysicsLoopSynchronizer::all_controllers_are_active() const
{
  return controllers_active_.load(std::memory_order_acquire);
}

void PhysicsLoopSynchronizer::initialize_physics_sync_node()
{
  rclcpp::NodeOptions node_options;
  node_options.use_global_arguments(false);
  synchronizer_node_ = std::make_shared<rclcpp::Node>("physics_sync_node", node_options);
  list_controllers_client_ =
    synchronizer_node_->create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers");
  pause_simulation_service_ = synchronizer_node_->create_service<std_srvs::srv::Trigger>(
    "~/pause_simulation",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      simulation_paused_.store(true, std::memory_order_release);
      response->success = true;
      response->message = "Physics simulation paused.";
    });
  resume_simulation_service_ = synchronizer_node_->create_service<std_srvs::srv::Trigger>(
    "~/resume_simulation",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      simulation_paused_.store(false, std::memory_order_release);
      response->success = true;
      response->message = "Physics simulation resumed.";
    });
  physics_sync_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  physics_sync_executor_->add_node(synchronizer_node_);
  physics_sync_executor_thread_ =
    std::thread([this]() { physics_sync_executor_->spin(); });
}

void PhysicsLoopSynchronizer::request_controller_states()
{
  if (controller_request_in_flight_.exchange(true, std::memory_order_acq_rel))
  {
    return;
  }

  if (!list_controllers_client_->service_is_ready())
  {
    controller_request_in_flight_.store(false, std::memory_order_release);
    return;
  }

  list_controllers_client_->async_send_request(
    std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
    [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
      bool all_active = false;
      try
      {
        const auto response = future.get();
        all_active = std::all_of(
          required_controller_names_.cbegin(), required_controller_names_.cend(),
          [&response](const std::string & required_name) {
            const auto controller = std::find_if(
              response->controller.cbegin(), response->controller.cend(),
              [&required_name](const auto & controller_state) {
                return controller_state.name == required_name;
              });
            return controller != response->controller.cend() && controller->state == "active";
          });
      }
      catch (const std::exception &)
      {
        all_active = false;
      }

      controllers_active_.store(all_active, std::memory_order_release);
      controller_request_in_flight_.store(false, std::memory_order_release);
    });
}

void PhysicsLoopSynchronizer::update_expected_write_time_loop()
{
  using namespace std::chrono_literals;
  set_current_thread_to_low_priority();

  rclcpp::Time observed_write_time(0, 0, RCL_ROS_TIME);
  auto next_controller_state_request = std::chrono::steady_clock::now();
  while (updater_running_.load())
  {
    if (std::chrono::steady_clock::now() >= next_controller_state_request)
    {
      request_controller_states();
      next_controller_state_request = std::chrono::steady_clock::now() + 20ms;
    }

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
