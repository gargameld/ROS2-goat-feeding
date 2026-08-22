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
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

namespace mujoco_ros2_control
{

class MujocoSimulation;

/**
 * @brief Waits for required controllers, then prevents physics from advancing
 * beyond the next expected ROS write.
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
   * @brief Yield until required controllers are active and the next expected
   * ROS write is no longer overdue.
   */
  void sync_physics_loop() const;

private:
  bool all_controllers_are_active() const;
  void initialize_controller_state_node();
  void request_controller_states();
  void update_expected_write_time_loop();

  MujocoSimulation* simulation_;
  const rclcpp::Time* last_ros_write_time_;
  std::mutex* last_ros_write_time_mutex_;

  const double write_period_seconds_;
  const double safety_time_interval_seconds_;
  const std::chrono::duration<double, std::milli> extra_wait_time_;
  const std::vector<std::string> required_controller_names_;

  rclcpp::Node::SharedPtr synchronizer_node_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> controller_state_executor_;
  std::atomic<bool> controllers_active_{ false };
  std::atomic<bool> controller_request_in_flight_{ false };
  mutable std::atomic<bool> initial_sync_completed_{ false };
  mutable std::atomic<bool> controller_activation_logged_{ false };
  std::thread controller_state_executor_thread_;

  mutable std::mutex expected_write_time_mutex_;
  rclcpp::Time next_expected_write_time_{ 0, 0, RCL_ROS_TIME };

  std::atomic<bool> updater_running_{ true };
  std::thread expected_write_time_thread_;
};

}  // namespace mujoco_ros2_control
