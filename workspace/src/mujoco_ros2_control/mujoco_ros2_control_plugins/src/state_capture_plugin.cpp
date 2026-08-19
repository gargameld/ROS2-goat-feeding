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

#include "mujoco_ros2_control_plugins/state_capture_plugin.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <limits>
#include <string_view>
#include <system_error>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

StateCapturePlugin::~StateCapturePlugin()
{
  cleanup();
}

bool StateCapturePlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  if (!node || !model)
  {
    return false;
  }

  node_ = std::move(node);
  logger_ = node_->get_logger().get_child("StateCapturePlugin");

  if (!node_->has_parameter("capture_rate"))
  {
    node_->declare_parameter("capture_rate", capture_rate_hz_);
  }
  if (!node_->has_parameter("flush_interval"))
  {
    node_->declare_parameter("flush_interval", flush_interval_seconds_);
  }
  if (!node_->has_parameter("buffer_capacity"))
  {
    node_->declare_parameter("buffer_capacity", static_cast<int64_t>(buffer_capacity_));
  }
  if (!node_->has_parameter("output_directory"))
  {
    node_->declare_parameter("output_directory", std::string("/config/workspace/capture"));
  }
  if (!node_->has_parameter("output_file"))
  {
    node_->declare_parameter("output_file", std::string("simulation_states.csv"));
  }
  if (!node_->has_parameter("food_body_prefix"))
  {
    node_->declare_parameter("food_body_prefix", food_body_prefix_);
  }

  capture_rate_hz_ = node_->get_parameter("capture_rate").as_double();
  flush_interval_seconds_ = node_->get_parameter("flush_interval").as_double();
  const int64_t configured_capacity = node_->get_parameter("buffer_capacity").as_int();
  const std::filesystem::path output_directory = node_->get_parameter("output_directory").as_string();
  const std::filesystem::path output_file = node_->get_parameter("output_file").as_string();
  food_body_prefix_ = node_->get_parameter("food_body_prefix").as_string();

  if (capture_rate_hz_ <= 0.0 || flush_interval_seconds_ <= 0.0 || configured_capacity < 2)
  {
    RCLCPP_ERROR(logger_, "capture_rate and flush_interval must be positive, and buffer_capacity must be at least 2.");
    return false;
  }
  if (output_file.empty() || output_file.has_parent_path())
  {
    RCLCPP_ERROR(logger_, "output_file must be a filename without directory components.");
    return false;
  }

  buffer_capacity_ = static_cast<std::size_t>(configured_capacity);
  nq_ = static_cast<std::size_t>(model->nq);
  output_path_ = output_directory / output_file;

  // Discover the free-floating STL food bodies by name prefix and record the
  // slice of qpos that stores each one's free-joint state.
  food_bodies_.clear();
  food_qpos_total_ = 0;
  if (!food_body_prefix_.empty())
  {
    for (int body_id = 0; body_id < model->nbody; ++body_id)
    {
      const char* body_name = mj_id2name(model, mjOBJ_BODY, body_id);
      if (!body_name || std::string_view(body_name).substr(0, food_body_prefix_.size()) != food_body_prefix_)
      {
        continue;
      }

      const int joint_count = model->body_jntnum[body_id];
      const int first_joint = model->body_jntadr[body_id];
      for (int joint = first_joint; joint < first_joint + joint_count; ++joint)
      {
        if (model->jnt_type[joint] != mjJNT_FREE)
        {
          continue;
        }
        FoodBody food_body;
        food_body.name = body_name;
        food_body.qpos_address = model->jnt_qposadr[joint];
        food_body.qpos_count = 7;  // free joint: 3 translation + 4 quaternion
        food_qpos_total_ += static_cast<std::size_t>(food_body.qpos_count);
        food_bodies_.push_back(std::move(food_body));
        break;  // one free joint per food body
      }
    }
  }

  std::error_code filesystem_error;
  std::filesystem::create_directories(output_directory, filesystem_error);
  if (filesystem_error)
  {
    RCLCPP_ERROR(logger_, "Failed to create capture directory '%s': %s", output_directory.string().c_str(),
                 filesystem_error.message().c_str());
    return false;
  }

  output_stream_.open(output_path_, std::ios::out | std::ios::trunc);
  if (!output_stream_.is_open())
  {
    RCLCPP_ERROR(logger_, "Failed to open capture file '%s'.", output_path_.string().c_str());
    return false;
  }

  output_stream_ << std::setprecision(std::numeric_limits<double>::max_digits10);
  output_stream_ << "time";
  for (std::size_t index = 0; index < nq_; ++index)
  {
    output_stream_ << ",qpos_" << index;
  }
  for (const FoodBody& food_body : food_bodies_)
  {
    for (int component = 0; component < food_body.qpos_count; ++component)
    {
      output_stream_ << ',' << food_body.name << "_qpos_" << component;
    }
  }
  output_stream_ << '\n';
  output_stream_.flush();

  if (!output_stream_)
  {
    RCLCPP_ERROR(logger_, "Failed to write capture header to '%s'.", output_path_.string().c_str());
    output_stream_.close();
    return false;
  }

  ring_buffer_.resize(buffer_capacity_);
  for (auto& sample : ring_buffer_)
  {
    sample.qpos.resize(nq_);
    sample.food_qpos.resize(food_qpos_total_);
  }

  write_sequence_.store(0, std::memory_order_relaxed);
  read_sequence_.store(0, std::memory_order_relaxed);
  dropped_samples_.store(0, std::memory_order_relaxed);
  capture_schedule_initialized_ = false;
  next_capture_time_ = 0.0;
  previous_simulation_time_ = 0.0;

  {
    const std::lock_guard<std::mutex> lock(consumer_mutex_);
    stop_requested_ = false;
  }

  capture_enabled_.store(true, std::memory_order_release);
  consumer_thread_ = std::thread(&StateCapturePlugin::consumer_loop, this);

  RCLCPP_INFO(logger_, "Capturing qpos at %.2f Hz to '%s'; flushing every %.2f seconds.", capture_rate_hz_,
              output_path_.string().c_str(), flush_interval_seconds_);
  RCLCPP_INFO(logger_, "Also logging qpos for %zu STL food body(ies) matching prefix '%s'.", food_bodies_.size(),
              food_body_prefix_.c_str());
  return true;
}

void StateCapturePlugin::update(const mjModel* /*model*/, mjData* data)
{
  if (!capture_enabled_.load(std::memory_order_acquire) || !data)
  {
    return;
  }

  const double simulation_time = static_cast<double>(data->time);
  const double capture_period = 1.0 / capture_rate_hz_;

  if (!capture_schedule_initialized_ || simulation_time < previous_simulation_time_)
  {
    next_capture_time_ = simulation_time;
    capture_schedule_initialized_ = true;
  }
  previous_simulation_time_ = simulation_time;

  if (simulation_time + 1e-12 < next_capture_time_)
  {
    return;
  }

  do
  {
    next_capture_time_ += capture_period;
  }
  while (next_capture_time_ <= simulation_time + 1e-12);

  const uint64_t write_sequence = write_sequence_.load(std::memory_order_relaxed);
  const uint64_t read_sequence = read_sequence_.load(std::memory_order_acquire);
  if (write_sequence - read_sequence >= buffer_capacity_)
  {
    dropped_samples_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  StateSample& sample = ring_buffer_[static_cast<std::size_t>(write_sequence % buffer_capacity_)];
  sample.simulation_time = simulation_time;
  for (std::size_t index = 0; index < nq_; ++index)
  {
    sample.qpos[index] = static_cast<double>(data->qpos[index]);
  }

  std::size_t food_offset = 0;
  for (const FoodBody& food_body : food_bodies_)
  {
    for (int component = 0; component < food_body.qpos_count; ++component)
    {
      sample.food_qpos[food_offset++] = static_cast<double>(data->qpos[food_body.qpos_address + component]);
    }
  }

  write_sequence_.store(write_sequence + 1, std::memory_order_release);
}

void StateCapturePlugin::consumer_loop()
{
  const auto flush_interval = std::chrono::duration<double>(flush_interval_seconds_);

  while (true)
  {
    bool stopping = false;
    {
      std::unique_lock<std::mutex> lock(consumer_mutex_);
      consumer_cv_.wait_for(lock, flush_interval, [this]() { return stop_requested_; });
      stopping = stop_requested_;
    }

    drain_buffer();

    if (stopping)
    {
      break;
    }
  }
}

void StateCapturePlugin::drain_buffer()
{
  uint64_t read_sequence = read_sequence_.load(std::memory_order_relaxed);
  const uint64_t available_sequence = write_sequence_.load(std::memory_order_acquire);

  while (read_sequence < available_sequence)
  {
    const StateSample& sample = ring_buffer_[static_cast<std::size_t>(read_sequence % buffer_capacity_)];
    output_stream_ << sample.simulation_time;
    for (const double position : sample.qpos)
    {
      output_stream_ << ',' << position;
    }
    for (const double food_position : sample.food_qpos)
    {
      output_stream_ << ',' << food_position;
    }
    output_stream_ << '\n';

    ++read_sequence;
    read_sequence_.store(read_sequence, std::memory_order_release);
  }

  output_stream_.flush();
  if (!output_stream_)
  {
    RCLCPP_ERROR(logger_, "Failed while writing capture file '%s'; capture has stopped.",
                 output_path_.string().c_str());
    capture_enabled_.store(false, std::memory_order_release);
  }
}

void StateCapturePlugin::cleanup()
{
  capture_enabled_.store(false, std::memory_order_release);

  {
    const std::lock_guard<std::mutex> lock(consumer_mutex_);
    stop_requested_ = true;
  }
  consumer_cv_.notify_all();

  if (consumer_thread_.joinable())
  {
    consumer_thread_.join();
  }

  if (output_stream_.is_open())
  {
    output_stream_.flush();
    output_stream_.close();
  }

  const uint64_t dropped_samples = dropped_samples_.load(std::memory_order_relaxed);
  if (node_ && dropped_samples > 0)
  {
    RCLCPP_WARN(logger_, "Dropped %llu capture sample(s) because the ring buffer was full.",
                static_cast<unsigned long long>(dropped_samples));
  }

  ring_buffer_.clear();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::StateCapturePlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
