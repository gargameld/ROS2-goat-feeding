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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__STATE_CAPTURE_CONSUMER_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__STATE_CAPTURE_CONSUMER_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control_plugins
{

class StateCaptureConsumer
{
public:
  struct StateSample
  {
    double simulation_time{ 0.0 };
    std::vector<double> qpos;
    std::vector<double> food_qpos;
    std::array<double, 3> obstacle_position{};
  };

  StateCaptureConsumer(std::ofstream& output_stream, std::filesystem::path output_path, rclcpp::Logger logger,
                       std::size_t buffer_capacity, std::size_t nq, std::size_t food_qpos_total,
                       bool capture_obstacle, double flush_interval_seconds);
  ~StateCaptureConsumer();

  StateCaptureConsumer(const StateCaptureConsumer&) = delete;
  StateCaptureConsumer& operator=(const StateCaptureConsumer&) = delete;

  void start();
  void stop();
  bool is_enabled() const;
  StateSample* try_acquire_sample();
  void publish_sample();

private:
  void consumer_loop();
  void drain_buffer();

  std::ofstream& output_stream_;
  std::filesystem::path output_path_;
  rclcpp::Logger logger_;
  std::size_t buffer_capacity_;
  std::size_t nq_;
  std::size_t food_qpos_total_;
  bool capture_obstacle_;
  double flush_interval_seconds_;

  std::vector<StateSample> ring_buffer_;
  std::atomic<uint64_t> write_sequence_{ 0 };
  std::atomic<uint64_t> read_sequence_{ 0 };
  std::atomic<uint64_t> dropped_samples_{ 0 };
  uint64_t pending_write_sequence_{ 0 };

  std::thread consumer_thread_;
  std::mutex consumer_mutex_;
  std::condition_variable consumer_cv_;
  bool stop_requested_{ false };
  bool started_{ false };
  std::atomic<bool> capture_enabled_{ false };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__STATE_CAPTURE_CONSUMER_HPP_
