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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__STATE_CAPTURE_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__STATE_CAPTURE_PLUGIN_HPP_

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

class StateCaptureConsumer;

class StateCapturePlugin : public MuJoCoROS2ControlPluginBase
{
public:
  StateCapturePlugin() = default;
  ~StateCapturePlugin() override;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  // A free-floating STL food body tracked by name prefix, and the slice of the
  // MuJoCo qpos array that stores its free-joint state.
  struct FoodBody
  {
    std::string name;
    int qpos_address{ 0 };
    int qpos_count{ 0 };
  };

  bool configure_parameters();
  void discover_food_bodies(const mjModel* model);
  bool initialize_output_file();
  void start_consumer();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("StateCapturePlugin") };

  std::filesystem::path output_path_;
  std::ofstream output_stream_;

  std::size_t buffer_capacity_{ 4096 };
  std::size_t nq_{ 0 };
  std::vector<FoodBody> food_bodies_;
  std::size_t food_qpos_total_{ 0 };
  std::string food_body_prefix_{ "food_" };
  std::string obstacle_geom_name_{ "obstacle" };
  int obstacle_geom_id_{ -1 };

  double capture_rate_hz_{ 30.0 };
  double flush_interval_seconds_{ 4.0 };
  double next_capture_time_{ 0.0 };
  double previous_simulation_time_{ 0.0 };
  bool capture_schedule_initialized_{ false };

  std::unique_ptr<StateCaptureConsumer> consumer_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__STATE_CAPTURE_PLUGIN_HPP_
