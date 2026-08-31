#include "mujoco_ros2_control_plugins/state_capture/state_capture_plugin.hpp"

#include "mujoco_ros2_control_plugins/state_capture/state_capture_consumer.hpp"

#include "mujoco_ros2_control_plugins/plugin_parameters.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <string_view>
#include <system_error>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

namespace
{

constexpr char kPluginName[] = "state_capture";

}  // namespace

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

  PluginParameters parameters(node_);
  int64_t buffer_capacity = static_cast<int64_t>(buffer_capacity_);
  std::string output_directory = "/config/workspace/capture";
  std::string output_file = "simulation_states.csv";
  if (!parameters.get_parameter(kPluginName, "capture_rate", capture_rate_hz_, capture_rate_hz_) ||
      !parameters.get_parameter(kPluginName, "flush_interval", flush_interval_seconds_,
                                flush_interval_seconds_) ||
      !parameters.get_parameter(kPluginName, "buffer_capacity", buffer_capacity, buffer_capacity) ||
      !parameters.get_parameter(kPluginName, "output_directory", output_directory, output_directory) ||
      !parameters.get_parameter(kPluginName, "output_file", output_file, output_file) ||
      !parameters.get_parameter(kPluginName, "food_body_prefix", food_body_prefix_, food_body_prefix_))
  {
    return false;
  }
  const std::filesystem::path output_filename(output_file);
  if (!std::isfinite(capture_rate_hz_) || capture_rate_hz_ <= 0.0 || !std::isfinite(flush_interval_seconds_) ||
      flush_interval_seconds_ <= 0.0 || buffer_capacity < 2 || output_filename.empty() ||
      output_filename.has_parent_path())
  {
    RCLCPP_ERROR(logger_, "Invalid state-capture parameter value.");
    return false;
  }
  buffer_capacity_ = static_cast<std::size_t>(buffer_capacity);
  output_path_ = std::filesystem::path(output_directory) / output_filename;

  nq_ = static_cast<std::size_t>(model->nq);
  discover_food_bodies(model);

  if (!initialize_output_file())
  {
    return false;
  }

  start_consumer();

  RCLCPP_INFO(logger_, "Capturing qpos at %.2f Hz to '%s'; flushing every %.2f seconds.", capture_rate_hz_,
              output_path_.string().c_str(), flush_interval_seconds_);
  RCLCPP_INFO(logger_, "Also logging qpos for %zu STL food body(ies) matching prefix '%s'.", food_bodies_.size(),
              food_body_prefix_.c_str());
  return true;
}

void StateCapturePlugin::discover_food_bodies(const mjModel* model)
{
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
}

bool StateCapturePlugin::initialize_output_file()
{
  std::error_code filesystem_error;
  std::filesystem::create_directories(output_path_.parent_path(), filesystem_error);
  if (filesystem_error)
  {
    RCLCPP_ERROR(logger_, "Failed to create capture directory '%s': %s", output_path_.parent_path().string().c_str(),
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

  return true;
}

void StateCapturePlugin::update(const mjModel* model, mjData* data)
{
  if (!consumer_ || !consumer_->is_enabled() || !model || !data)
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

  StateCaptureConsumer::StateSample* sample = consumer_->try_acquire_sample();
  if (!sample)
  {
    return;
  }

  sample->simulation_time = simulation_time;
  for (std::size_t index = 0; index < nq_; ++index)
  {
    sample->qpos[index] = static_cast<double>(data->qpos[index]);
  }
  std::size_t food_offset = 0;
  for (const FoodBody& food_body : food_bodies_)
  {
    for (int component = 0; component < food_body.qpos_count; ++component)
    {
      sample->food_qpos[food_offset++] = static_cast<double>(data->qpos[food_body.qpos_address + component]);
    }
  }

  consumer_->publish_sample();
}

void StateCapturePlugin::cleanup()
{
  if (consumer_)
  {
    consumer_->stop();
    consumer_.reset();
  }

  if (output_stream_.is_open())
  {
    output_stream_.flush();
    output_stream_.close();
  }

  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::StateCapturePlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
