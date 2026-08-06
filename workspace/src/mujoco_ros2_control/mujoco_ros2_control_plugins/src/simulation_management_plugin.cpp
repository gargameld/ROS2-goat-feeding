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

#include "mujoco_ros2_control_plugins/simulation_management_plugin.hpp"

#include <functional>
#include <mutex>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

bool SimulationManagementPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data)
{
  if (!node || !model || !data)
  {
    return false;
  }

  node_ = std::move(node);
  logger_ = node_->get_logger().get_child("SimulationManagementPlugin");

  if (!simulation_mutex())
  {
    RCLCPP_ERROR(logger_, "The simulation mutex was not provided to SimulationManagementPlugin.");
    cleanup();
    return false;
  }

  data_ = data;
  nq_ = static_cast<std::size_t>(model->nq);

  get_robot_state_service_ = node_->create_service<GetRobotState>(
      "get_robot_state", std::bind(&SimulationManagementPlugin::handle_get_robot_state, this,
                                   std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(logger_, "SimulationManagementPlugin initialized. Service available at '%s'.",
              get_robot_state_service_->get_service_name());
  return true;
}

void SimulationManagementPlugin::update(const mjModel* /*model*/, mjData* /*data*/)
{
}

void SimulationManagementPlugin::cleanup()
{
  get_robot_state_service_.reset();
  data_ = nullptr;
  nq_ = 0;
  node_.reset();
}

void SimulationManagementPlugin::handle_get_robot_state(const GetRobotState::Request::SharedPtr /*request*/,
                                                        GetRobotState::Response::SharedPtr response)
{
  auto* mutex = simulation_mutex();
  if (!mutex || !data_)
  {
    RCLCPP_ERROR(logger_, "Cannot get the robot state because the simulation is unavailable.");
    return;
  }

  const std::unique_lock<std::recursive_mutex> lock(*mutex);
  response->qpos.resize(nq_);
  for (std::size_t index = 0; index < nq_; ++index)
  {
    response->qpos[index] = static_cast<double>(data_->qpos[index]);
  }
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::SimulationManagementPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
