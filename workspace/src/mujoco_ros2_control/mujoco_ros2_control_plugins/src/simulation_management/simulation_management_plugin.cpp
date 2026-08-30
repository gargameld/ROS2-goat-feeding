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

#include "mujoco_ros2_control_plugins/simulation_management/simulation_management_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

namespace
{

bool load_parking_frames(const rclcpp::Node::SharedPtr& node, const std::string& param_prefix, int parking_count,
                         std::vector<ParkingFrame>& parking_frames, std::string& error)
{
  if (parking_count <= 0)
  {
    error = "parking_count must be greater than zero.";
    return false;
  }

  parking_frames.clear();
  parking_frames.reserve(static_cast<std::size_t>(parking_count));
  for (int parking_index = 1; parking_index <= parking_count; ++parking_index)
  {
    const std::string parking_prefix =
        param_prefix + "parking_frames.parking_" + std::to_string(parking_index) + ".";
    const std::string offset_param = parking_prefix + "offset";
    const std::string min_x_param = parking_prefix + "min_x";
    const std::string max_x_param = parking_prefix + "max_x";
    const std::string min_y_param = parking_prefix + "min_y";
    const std::string max_y_param = parking_prefix + "max_y";

    const std::vector<std::string> required_parameters = {
      offset_param, min_x_param, max_x_param, min_y_param, max_y_param
    };
    for (const std::string& parameter : required_parameters)
    {
      if (!node->has_parameter(parameter))
      {
        error = "Required parking configuration parameter '" + parameter + "' is missing.";
        return false;
      }
    }

    const std::vector<double> offset = node->get_parameter(offset_param).as_double_array();
    if (offset.size() != 2)
    {
      error = "Parameter '" + offset_param + "' must contain exactly two values: [x, y].";
      return false;
    }

    ParkingFrame frame{ offset[0], offset[1], node->get_parameter(min_x_param).as_double(),
                        node->get_parameter(max_x_param).as_double(), node->get_parameter(min_y_param).as_double(),
                        node->get_parameter(max_y_param).as_double() };
    const double values[] = { frame.offset_x, frame.offset_y, frame.min_x,
                              frame.max_x,   frame.min_y,    frame.max_y };
    if (!std::all_of(std::begin(values), std::end(values), [](double value) { return std::isfinite(value); }))
    {
      error = "Parking " + std::to_string(parking_index) + " configuration must contain only finite values.";
      return false;
    }
    if (frame.min_x > frame.max_x || frame.min_y > frame.max_y)
    {
      error = "Parking " + std::to_string(parking_index) +
              " configuration requires min_x <= max_x and min_y <= max_y.";
      return false;
    }
    parking_frames.push_back(frame);
  }
  return true;
}

}  // namespace

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
  model_ = const_cast<mjModel*>(model);
  nq_ = static_cast<std::size_t>(model->nq);
  obstacle_management_ = std::make_unique<ObstacleManagement>(mujoco_spec(), model_, data_);
  if (!obstacle_management_->is_available())
  {
    RCLCPP_ERROR(logger_, "The editable MuJoCo specification does not contain the named 'obstacle' box geom.");
    cleanup();
    return false;
  }

  // Parameters are nested below the configured ROS plugin key. The plugin is given a sub-node whose
  // sub-namespace is that key, but sub-namespaces only affect topic and service names, so the
  // parameter names have to carry the prefix explicitly.
  const std::string plugin_key =
      node_->get_sub_namespace().empty() ? std::string("simulation_management") : node_->get_sub_namespace();
  const std::string param_prefix = "mujoco_plugins." + plugin_key + ".";
  const std::string throw_food_height_param = param_prefix + "throw_food_height";
  const std::string parking_count_param = param_prefix + "parking_count";

  if (!node_->has_parameter(throw_food_height_param))
  {
    node_->declare_parameter(throw_food_height_param, 0.3);
  }
  if (!node_->has_parameter(parking_count_param))
  {
    node_->declare_parameter(parking_count_param, static_cast<int64_t>(4));
  }
  const double throw_food_height = node_->get_parameter(throw_food_height_param).as_double();
  const int parking_count = static_cast<int>(node_->get_parameter(parking_count_param).as_int());
  std::vector<ParkingFrame> parking_frames;
  std::string parking_configuration_error;
  if (!load_parking_frames(node_, param_prefix, parking_count, parking_frames, parking_configuration_error))
  {
    RCLCPP_ERROR(logger_, "Invalid parking-frame configuration: %s", parking_configuration_error.c_str());
    cleanup();
    return false;
  }
  RCLCPP_INFO(logger_, "Using '%s' = %.3f and '%s' = %d.", throw_food_height_param.c_str(), throw_food_height,
              parking_count_param.c_str(), parking_count);
  food_management_ =
      std::make_unique<FoodManagement>(model_, data_, throw_food_height, std::move(parking_frames));

  get_robot_state_service_ = node_->create_service<GetRobotState>(
      "get_robot_state", std::bind(&SimulationManagementPlugin::handle_get_robot_state, this,
                                   std::placeholders::_1, std::placeholders::_2));
  set_obstacle_service_ = node_->create_service<SetObstacle>(
      "set_obstacle", std::bind(&SimulationManagementPlugin::handle_set_obstacle, this, std::placeholders::_1,
                                std::placeholders::_2));
  throw_food_service_ = node_->create_service<ThrowFood>(
      "throw_food", std::bind(&SimulationManagementPlugin::handle_throw_food, this, std::placeholders::_1,
                              std::placeholders::_2));

  RCLCPP_INFO(logger_, "SimulationManagementPlugin initialized. Services available at '%s', '%s' and '%s'.",
              get_robot_state_service_->get_service_name(), set_obstacle_service_->get_service_name(),
              throw_food_service_->get_service_name());
  return true;
}

void SimulationManagementPlugin::update(const mjModel* /*model*/, mjData* /*data*/)
{
}

void SimulationManagementPlugin::cleanup()
{
  throw_food_service_.reset();
  set_obstacle_service_.reset();
  get_robot_state_service_.reset();
  food_management_.reset();
  obstacle_management_.reset();
  model_ = nullptr;
  data_ = nullptr;
  nq_ = 0;
  node_.reset();
}

void SimulationManagementPlugin::handle_get_robot_state(const GetRobotState::Request::SharedPtr /*request*/,
                                                        GetRobotState::Response::SharedPtr response)
{
  auto* mutex = simulation_mutex();
  if (!mutex || !data_ || !obstacle_management_)
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
  const ObstacleState obstacle = obstacle_management_->state();
  response->obstacle_position.x = obstacle.x;
  response->obstacle_position.y = obstacle.y;
  response->obstacle_position.z = obstacle.z;
  response->obstacle_size.x = obstacle.width;
  response->obstacle_size.y = obstacle.length;
  response->obstacle_size.z = obstacle.height;
}

void SimulationManagementPlugin::handle_set_obstacle(const SetObstacle::Request::SharedPtr request,
                                                      SetObstacle::Response::SharedPtr response)
{
  auto* mutex = simulation_mutex();
  if (!mutex || !obstacle_management_)
  {
    response->message = "The MuJoCo simulation is unavailable.";
    RCLCPP_ERROR(logger_, "%s", response->message.c_str());
    return;
  }

  const std::unique_lock<std::recursive_mutex> lock(*mutex);
  std::string error;
  response->success = obstacle_management_->set_obstacle(request->position.x, request->position.y, error);
  response->message = response->success ? "Obstacle updated." : error;
  if (!response->success)
  {
    RCLCPP_WARN(logger_, "Could not update obstacle: %s", error.c_str());
  }
}

void SimulationManagementPlugin::handle_throw_food(const ThrowFood::Request::SharedPtr request,
                                                   ThrowFood::Response::SharedPtr response)
{
  response->success = false;
  auto* mutex = simulation_mutex();
  if (!mutex || !food_management_)
  {
    response->message = "The MuJoCo simulation is unavailable.";
    RCLCPP_ERROR(logger_, "%s", response->message.c_str());
    return;
  }
  if (request->orientation.size() != 4)
  {
    response->message = "The orientation must contain exactly 4 quaternion values (w, x, y, z).";
    RCLCPP_WARN(logger_, "%s", response->message.c_str());
    return;
  }

  const double quat[4] = { request->orientation[0], request->orientation[1], request->orientation[2],
                           request->orientation[3] };

  const std::unique_lock<std::recursive_mutex> lock(*mutex);
  std::string error;
  response->success =
      food_management_->throw_food(request->parking_index, request->food_name, request->x, request->y, quat, error);
  response->message = response->success ? "Food thrown." : error;
  if (!response->success)
  {
    RCLCPP_WARN(logger_, "Could not throw food: %s", error.c_str());
  }
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::SimulationManagementPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
