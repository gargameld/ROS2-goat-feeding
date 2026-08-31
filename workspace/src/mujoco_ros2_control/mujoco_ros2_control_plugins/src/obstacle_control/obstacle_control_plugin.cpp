#include "mujoco_ros2_control_plugins/obstacle_control/obstacle_control_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
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

constexpr char kSetObstacleService[] = "/simulation_management/set_obstacle";
constexpr char kPluginName[] = "obstacle_control";

int find_free_joint(const mjModel* model, const int body_id)
{
  const int first_joint = model->body_jntadr[body_id];
  const int joint_count = model->body_jntnum[body_id];
  for (int joint = first_joint; joint < first_joint + joint_count; ++joint)
  {
    if (model->jnt_type[joint] == mjJNT_FREE)
    {
      return joint;
    }
  }
  return -1;
}

}  // namespace

bool ObstacleControlPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data)
{
  if (!node || !model || !data || !simulation_mutex())
  {
    return false;
  }

  node_ = std::move(node);
  logger_ = node_->get_logger().get_child("ObstacleControlPlugin");
  model_ = const_cast<mjModel*>(model);
  data_ = data;

  PluginParameters parameters(node_);
  std::string body_name;
  std::vector<double> initial_position;
  if (!parameters.get_parameter(kPluginName, "body_name", std::string("obstacle"), body_name) ||
      !parameters.get_parameter(kPluginName, "initial_position", initial_position))
  {
    cleanup();
    return false;
  }
  if (body_name.empty() || initial_position.size() != stored_position_.size() ||
      !std::all_of(initial_position.begin(), initial_position.end(), [](double value) { return std::isfinite(value); }))
  {
    RCLCPP_ERROR(logger_, "Obstacle body_name must not be empty and initial_position must be [x, y, z].");
    cleanup();
    return false;
  }

  const int body_id = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
  if (body_id < 0)
  {
    RCLCPP_ERROR(logger_, "No obstacle body named '%s' exists in the MuJoCo model.", body_name.c_str());
    cleanup();
    return false;
  }
  const int joint_id = find_free_joint(model_, body_id);
  if (joint_id < 0)
  {
    RCLCPP_ERROR(logger_, "Obstacle body '%s' must have a free joint.", body_name.c_str());
    cleanup();
    return false;
  }

  qpos_address_ = model_->jnt_qposadr[joint_id];
  dof_address_ = model_->jnt_dofadr[joint_id];
  std::copy(initial_position.begin(), initial_position.end(), stored_position_.begin());
  std::copy_n(data_->qpos + qpos_address_ + 3, initial_orientation_.size(), initial_orientation_.begin());

  set_obstacle_service_ = node_->create_service<SetObstacle>(
      kSetObstacleService, std::bind(&ObstacleControlPlugin::handle_set_obstacle, this, std::placeholders::_1,
                                     std::placeholders::_2));

  RCLCPP_INFO(logger_, "Controlling free obstacle body '%s' from '%s'.", body_name.c_str(),
              set_obstacle_service_->get_service_name());
  return true;
}

void ObstacleControlPlugin::update(const mjModel* /*model*/, mjData* data)
{
  auto* mutex = simulation_mutex();
  if (!mutex || !model_ || !data_ || qpos_address_ < 0)
  {
    return;
  }

  const std::unique_lock<std::recursive_mutex> lock(*mutex);
  apply_stored_pose(data_);
  mj_forward(model_, data_);

  // Keep the controller-facing copy coherent immediately as well. The system
  // interface passes that second mjData to update(), while init() receives the
  // live simulation data.
  if (data && data != data_)
  {
    apply_stored_pose(data);
    mj_forward(model_, data);
  }
}

void ObstacleControlPlugin::cleanup()
{
  set_obstacle_service_.reset();
  model_ = nullptr;
  data_ = nullptr;
  qpos_address_ = -1;
  dof_address_ = -1;
  node_.reset();
}

void ObstacleControlPlugin::apply_stored_pose(mjData* data) const
{
  std::copy(stored_position_.begin(), stored_position_.end(), data->qpos + qpos_address_);
  std::copy(initial_orientation_.begin(), initial_orientation_.end(), data->qpos + qpos_address_ + 3);
  std::fill_n(data->qvel + dof_address_, 6, 0.0);
}

void ObstacleControlPlugin::handle_set_obstacle(const SetObstacle::Request::SharedPtr request,
                                                SetObstacle::Response::SharedPtr response)
{
  response->success = false;
  auto* mutex = simulation_mutex();
  if (!mutex || !data_)
  {
    response->message = "The MuJoCo simulation is unavailable.";
    RCLCPP_ERROR(logger_, "%s", response->message.c_str());
    return;
  }

  const double requested_xy[] = { request->position.x, request->position.y };
  if (!std::all_of(std::begin(requested_xy), std::end(requested_xy),
                   [](double value) { return std::isfinite(value); }))
  {
    response->message = "Obstacle position must be finite.";
    RCLCPP_WARN(logger_, "%s", response->message.c_str());
    return;
  }

  const std::unique_lock<std::recursive_mutex> lock(*mutex);
  stored_position_[0] = request->position.x;
  stored_position_[1] = request->position.y;
  response->success = true;
  response->message = "Obstacle updated.";
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::ObstacleControlPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
