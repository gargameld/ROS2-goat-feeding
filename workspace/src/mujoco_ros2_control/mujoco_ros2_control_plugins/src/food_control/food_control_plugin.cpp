#include "mujoco_ros2_control_plugins/food_control/food_control_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "mujoco_ros2_control_plugins/plugin_parameters.hpp"

namespace mujoco_ros2_control_plugins
{

namespace
{

constexpr char kThrowFoodService[] = "/simulation_management/throw_food";
constexpr char kPluginName[] = "food_control";

bool all_finite(const std::vector<double>& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

bool get_parking_frames(const PluginParameters& parameters, const rclcpp::Logger& logger,
                        std::vector<ParkingFrame>& parking_frames)
{
  int64_t parking_count = 4;
  if (!parameters.get_parameter(kPluginName, "parking_count", parking_count, parking_count))
  {
    return false;
  }
  if (parking_count <= 0)
  {
    RCLCPP_ERROR(logger, "parking_count must be greater than zero.");
    return false;
  }

  parking_frames.clear();
  parking_frames.reserve(static_cast<std::size_t>(parking_count));
  for (int64_t parking_index = 1; parking_index <= parking_count; ++parking_index)
  {
    const std::string prefix = "parking_frames.parking_" + std::to_string(parking_index) + ".";
    std::vector<double> offset;
    ParkingFrame frame{};
    if (!parameters.get_parameter(kPluginName, prefix + "offset", offset) ||
        !parameters.get_parameter(kPluginName, prefix + "min_x", frame.min_x) ||
        !parameters.get_parameter(kPluginName, prefix + "max_x", frame.max_x) ||
        !parameters.get_parameter(kPluginName, prefix + "min_y", frame.min_y) ||
        !parameters.get_parameter(kPluginName, prefix + "max_y", frame.max_y))
    {
      return false;
    }
    if (offset.size() != 2 || !all_finite(offset) ||
        !all_finite({ frame.min_x, frame.max_x, frame.min_y, frame.max_y }) || frame.min_x > frame.max_x ||
        frame.min_y > frame.max_y)
    {
      RCLCPP_ERROR(logger, "Parking frame %ld has invalid offsets or limits.", parking_index);
      return false;
    }
    frame.offset_x = offset[0];
    frame.offset_y = offset[1];
    parking_frames.push_back(frame);
  }

  return true;
}

}  // namespace

bool FoodControlPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data)
{
  if (!node || !model || !data || !simulation_mutex())
  {
    return false;
  }

  node_ = std::move(node);
  logger_ = node_->get_logger().get_child("FoodControlPlugin");

  PluginParameters parameters(node_);
  double throw_food_height = 0.3;
  if (!parameters.get_parameter(kPluginName, "throw_food_height", throw_food_height, throw_food_height))
  {
    cleanup();
    return false;
  }
  if (!std::isfinite(throw_food_height))
  {
    RCLCPP_ERROR(logger_, "throw_food_height must be finite.");
    cleanup();
    return false;
  }

  std::vector<ParkingFrame> parking_frames;
  if (!get_parking_frames(parameters, logger_, parking_frames))
  {
    cleanup();
    return false;
  }

  food_management_ = std::make_unique<FoodManagement>(const_cast<mjModel*>(model), data, throw_food_height,
                                                      std::move(parking_frames));
  throw_food_service_ = node_->create_service<ThrowFood>(
      kThrowFoodService, std::bind(&FoodControlPlugin::handle_throw_food, this, std::placeholders::_1,
                                   std::placeholders::_2));

  RCLCPP_INFO(logger_, "FoodControlPlugin initialized. Service available at '%s'.",
              throw_food_service_->get_service_name());
  return true;
}

void FoodControlPlugin::update(const mjModel* /*model*/, mjData* /*data*/)
{
}

void FoodControlPlugin::cleanup()
{
  throw_food_service_.reset();
  food_management_.reset();
  node_.reset();
}

void FoodControlPlugin::handle_throw_food(const ThrowFood::Request::SharedPtr request,
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

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::FoodControlPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
