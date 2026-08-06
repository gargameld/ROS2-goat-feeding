// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__SIMULATION_MANAGEMENT_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__SIMULATION_MANAGEMENT_PLUGIN_HPP_

#include <cstddef>
#include <memory>

#include <mujoco_ros2_control_msgs/srv/get_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Provides services for inspecting and managing the MuJoCo simulation.
 */
class SimulationManagementPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  SimulationManagementPlugin() = default;
  ~SimulationManagementPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using GetRobotState = mujoco_ros2_control_msgs::srv::GetRobotState;

  void handle_get_robot_state(const GetRobotState::Request::SharedPtr request,
                              GetRobotState::Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("SimulationManagementPlugin") };
  rclcpp::Service<GetRobotState>::SharedPtr get_robot_state_service_;

  mjData* data_{ nullptr };
  std::size_t nq_{ 0 };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__SIMULATION_MANAGEMENT_PLUGIN_HPP_
