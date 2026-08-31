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

#include "mujoco_ros2_control_plugins/state_capture/state_capture_consumer.hpp"

#include "mujoco_ros2_control_plugins/state_capture/state_capture_plugin.hpp"

#include <chrono>
#include <utility>

namespace mujoco_ros2_control_plugins
{

StateCaptureConsumer::StateCaptureConsumer(std::ofstream& output_stream, std::filesystem::path output_path,
                                           rclcpp::Logger logger, const std::size_t buffer_capacity,
                                           const std::size_t nq, const std::size_t food_qpos_total,
                                           const double flush_interval_seconds)
  : output_stream_(output_stream),
    output_path_(std::move(output_path)),
    logger_(std::move(logger)),
    buffer_capacity_(buffer_capacity),
    nq_(nq),
    food_qpos_total_(food_qpos_total),
    flush_interval_seconds_(flush_interval_seconds)
{
}

StateCaptureConsumer::~StateCaptureConsumer()
{
  stop();
}

void StateCaptureConsumer::start()
{
  ring_buffer_.resize(buffer_capacity_);
  for (auto& sample : ring_buffer_)
  {
    sample.qpos.resize(nq_);
    sample.food_qpos.resize(food_qpos_total_);
  }

  write_sequence_.store(0, std::memory_order_relaxed);
  read_sequence_.store(0, std::memory_order_relaxed);
  dropped_samples_.store(0, std::memory_order_relaxed);
  pending_write_sequence_ = 0;

  {
    const std::lock_guard<std::mutex> lock(consumer_mutex_);
    stop_requested_ = false;
  }

  capture_enabled_.store(true, std::memory_order_release);
  consumer_thread_ = std::thread(&StateCaptureConsumer::consumer_loop, this);
  started_ = true;
}

void StateCaptureConsumer::stop()
{
  if (!started_)
  {
    return;
  }

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
  started_ = false;

  const uint64_t dropped_samples = dropped_samples_.load(std::memory_order_relaxed);
  if (dropped_samples > 0)
  {
    RCLCPP_WARN(logger_, "Dropped %llu capture sample(s) because the ring buffer was full.",
                static_cast<unsigned long long>(dropped_samples));
  }

  ring_buffer_.clear();
}

bool StateCaptureConsumer::is_enabled() const
{
  return capture_enabled_.load(std::memory_order_acquire);
}

StateCaptureConsumer::StateSample* StateCaptureConsumer::try_acquire_sample()
{
  if (!is_enabled())
  {
    return nullptr;
  }

  const uint64_t write_sequence = write_sequence_.load(std::memory_order_relaxed);
  const uint64_t read_sequence = read_sequence_.load(std::memory_order_acquire);
  if (write_sequence - read_sequence >= buffer_capacity_)
  {
    dropped_samples_.fetch_add(1, std::memory_order_relaxed);
    return nullptr;
  }

  pending_write_sequence_ = write_sequence;
  return &ring_buffer_[static_cast<std::size_t>(write_sequence % buffer_capacity_)];
}

void StateCaptureConsumer::publish_sample()
{
  write_sequence_.store(pending_write_sequence_ + 1, std::memory_order_release);
}

void StateCaptureConsumer::consumer_loop()
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

void StateCaptureConsumer::drain_buffer()
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

void StateCapturePlugin::start_consumer()
{
  capture_schedule_initialized_ = false;
  next_capture_time_ = 0.0;
  previous_simulation_time_ = 0.0;

  consumer_ = std::make_unique<StateCaptureConsumer>(output_stream_, output_path_, logger_, buffer_capacity_, nq_,
                                                      food_qpos_total_, flush_interval_seconds_);
  consumer_->start();
}

}  // namespace mujoco_ros2_control_plugins
