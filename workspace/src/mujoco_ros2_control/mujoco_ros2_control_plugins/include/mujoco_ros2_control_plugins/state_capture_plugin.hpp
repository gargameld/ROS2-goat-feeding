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

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

class StateCapturePlugin : public MuJoCoROS2ControlPluginBase
{
public:
  StateCapturePlugin() = default;
  ~StateCapturePlugin() override;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  struct StateSample
  {
    double simulation_time{ 0.0 };
    std::vector<double> qpos;
    // Free-joint qpos values of the tracked STL food bodies, concatenated in
    // food_bodies_ order (7 values per free joint: px, py, pz, qw, qx, qy, qz).
    std::vector<double> food_qpos;
  };

  // A free-floating STL food body tracked by name prefix, and the slice of the
  // MuJoCo qpos array that stores its free-joint state.
  struct FoodBody
  {
    std::string name;
    int qpos_address{ 0 };
    int qpos_count{ 0 };
  };

  void consumer_loop();
  void drain_buffer();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("StateCapturePlugin") };

  std::filesystem::path output_path_;
  std::ofstream output_stream_;

  std::vector<StateSample> ring_buffer_;
  std::size_t buffer_capacity_{ 4096 };
  std::size_t nq_{ 0 };
  std::vector<FoodBody> food_bodies_;
  std::size_t food_qpos_total_{ 0 };
  std::string food_body_prefix_{ "food_" };
  std::atomic<uint64_t> write_sequence_{ 0 };
  std::atomic<uint64_t> read_sequence_{ 0 };
  std::atomic<uint64_t> dropped_samples_{ 0 };

  double capture_rate_hz_{ 30.0 };
  double flush_interval_seconds_{ 4.0 };
  double next_capture_time_{ 0.0 };
  double previous_simulation_time_{ 0.0 };
  bool capture_schedule_initialized_{ false };

  std::thread consumer_thread_;
  std::mutex consumer_mutex_;
  std::condition_variable consumer_cv_;
  bool stop_requested_{ false };
  std::atomic<bool> capture_enabled_{ false };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__STATE_CAPTURE_PLUGIN_HPP_
