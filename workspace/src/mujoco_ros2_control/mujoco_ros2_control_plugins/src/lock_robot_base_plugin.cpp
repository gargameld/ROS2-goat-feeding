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

#include "mujoco_ros2_control_plugins/lock_robot_base_plugin.hpp"

#include <functional>
#include <mutex>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{
namespace
{

constexpr const char* kDefaultBodyName = "chassis";

}  // namespace

bool LockRobotBasePlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data)
{
  if (!node || !model || !data)
  {
    return false;
  }

  node_ = std::move(node);
  logger_ = node_->get_logger().get_child("LockRobotBasePlugin");

  if (!simulation_mutex())
  {
    RCLCPP_ERROR(logger_, "The simulation mutex was not provided to LockRobotBasePlugin.");
    cleanup();
    return false;
  }

  // Parameters are nested below the configured ROS plugin key. The plugin is given a sub-node whose
  // sub-namespace is that key, but sub-namespaces only affect topic and service names, so the
  // parameter names have to carry the prefix explicitly.
  const std::string plugin_key =
      node_->get_sub_namespace().empty() ? std::string("lock_robot_base") : node_->get_sub_namespace();
  const std::string body_name_param = "mujoco_plugins." + plugin_key + ".body_name";
  if (!node_->has_parameter(body_name_param))
  {
    node_->declare_parameter(body_name_param, std::string(kDefaultBodyName));
  }
  body_name_ = node_->get_parameter(body_name_param).as_string();

  const int body_id = mj_name2id(model, mjOBJ_BODY, body_name_.c_str());
  if (body_id < 0)
  {
    RCLCPP_ERROR(logger_, "The MuJoCo model does not contain a body named '%s'.", body_name_.c_str());
    cleanup();
    return false;
  }

  // A free-floating base carries exactly one free joint; that joint owns the 7 qpos values
  // (position and quaternion) and the 6 dofs that have to be pinned.
  for (int index = 0; index < model->body_jntnum[body_id]; ++index)
  {
    const int joint_id = model->body_jntadr[body_id] + index;
    if (model->jnt_type[joint_id] == mjJNT_FREE)
    {
      qpos_address_ = model->jnt_qposadr[joint_id];
      dof_address_ = model->jnt_dofadr[joint_id];
      break;
    }
  }
  if (qpos_address_ < 0)
  {
    RCLCPP_ERROR(logger_, "Body '%s' has no free joint, so its pose cannot be locked.", body_name_.c_str());
    cleanup();
    return false;
  }

  physics_data_ = data;

  lock_base_service_ = node_->create_service<Trigger>(
      "lock_base",
      std::bind(&LockRobotBasePlugin::handle_lock_base, this, std::placeholders::_1, std::placeholders::_2));
  unlock_base_service_ = node_->create_service<Trigger>(
      "unlock_base",
      std::bind(&LockRobotBasePlugin::handle_unlock_base, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(logger_, "LockRobotBasePlugin initialized for body '%s'. Services available at '%s' and '%s'.",
              body_name_.c_str(), lock_base_service_->get_service_name(),
              unlock_base_service_->get_service_name());
  return true;
}

void LockRobotBasePlugin::update(const mjModel* /*model*/, mjData* data)
{
  if (!locked_.load(std::memory_order_acquire))
  {
    return;
  }

  auto* mutex = simulation_mutex();
  if (!mutex || !physics_data_)
  {
    return;
  }

  const std::unique_lock<std::recursive_mutex> lock(*mutex);
  // Only the physics data feeds the next mj_step; the controller-facing copy handed in here is
  // rewritten from it after every step. Pinning both keeps the state reported for this control
  // cycle consistent with the pose the simulation will actually hold.
  apply_locked_pose(physics_data_);
  apply_locked_pose(data);
}

void LockRobotBasePlugin::cleanup()
{
  locked_.store(false, std::memory_order_release);
  unlock_base_service_.reset();
  lock_base_service_.reset();
  physics_data_ = nullptr;
  qpos_address_ = -1;
  dof_address_ = -1;
  node_.reset();
}

void LockRobotBasePlugin::snapshot_pose()
{
  for (std::size_t index = 0; index < locked_pose_.size(); ++index)
  {
    locked_pose_[index] = physics_data_->qpos[qpos_address_ + static_cast<int>(index)];
  }
}

void LockRobotBasePlugin::apply_locked_pose(mjData* data) const
{
  for (std::size_t index = 0; index < locked_pose_.size(); ++index)
  {
    data->qpos[qpos_address_ + static_cast<int>(index)] = locked_pose_[index];
  }
  // Without clearing the free-joint dofs the integrator would carry the accumulated velocity into
  // the next step and the base would visibly jitter around the pinned pose.
  for (int index = 0; index < 6; ++index)
  {
    data->qvel[dof_address_ + index] = 0.0;
    data->qacc[dof_address_ + index] = 0.0;
    data->qacc_warmstart[dof_address_ + index] = 0.0;
  }
}

void LockRobotBasePlugin::handle_lock_base(const Trigger::Request::SharedPtr /*request*/,
                                           Trigger::Response::SharedPtr response)
{
  auto* mutex = simulation_mutex();
  if (!mutex || !physics_data_)
  {
    response->message = "The MuJoCo simulation is unavailable.";
    RCLCPP_ERROR(logger_, "%s", response->message.c_str());
    return;
  }

  {
    const std::unique_lock<std::recursive_mutex> lock(*mutex);
    snapshot_pose();
  }
  locked_.store(true, std::memory_order_release);

  response->success = true;
  response->message = "Base locked.";
  RCLCPP_INFO(logger_, "Locked body '%s' at position [%.3f, %.3f, %.3f].", body_name_.c_str(), locked_pose_[0],
              locked_pose_[1], locked_pose_[2]);
}

void LockRobotBasePlugin::handle_unlock_base(const Trigger::Request::SharedPtr /*request*/,
                                             Trigger::Response::SharedPtr response)
{
  locked_.store(false, std::memory_order_release);
  response->success = true;
  response->message = "Base unlocked.";
  RCLCPP_INFO(logger_, "Unlocked body '%s'.", body_name_.c_str());
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::LockRobotBasePlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
