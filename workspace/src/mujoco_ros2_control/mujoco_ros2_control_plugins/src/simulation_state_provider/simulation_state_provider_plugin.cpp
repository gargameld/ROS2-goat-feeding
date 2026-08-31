// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mujoco_ros2_control_plugins/simulation_state_provider/simulation_state_provider_plugin.hpp"

#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "mujoco_ros2_control_plugins/plugin_parameters.hpp"

namespace mujoco_ros2_control_plugins
{

namespace
{

constexpr char kGetRobotStateService[] = "/simulation_management/get_robot_state";
constexpr char kPluginName[] = "simulation_state_provider";
constexpr char kArmAttachmentSiteName[] = "attachment_site";
constexpr std::array<const char*, 6> kArmBodyNames = { {
  "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"
} };

}  // namespace

bool SimulationStateProviderPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data)
{
  if (!node || !model || !data || !simulation_mutex())
  {
    return false;
  }

  node_ = std::move(node);
  logger_ = node_->get_logger().get_child("SimulationStateProviderPlugin");
  model_ = model;
  data_ = data;
  nq_ = static_cast<std::size_t>(model_->nq);

  PluginParameters parameters(node_);
  std::string obstacle_geom_name;
  if (!parameters.get_parameter(kPluginName, "obstacle_geom_name", std::string("obstacle"), obstacle_geom_name))
  {
    cleanup();
    return false;
  }

  for (std::size_t index = 0; index < kArmBodyNames.size(); ++index)
  {
    arm_body_ids_[index] = mj_name2id(model_, mjOBJ_BODY, kArmBodyNames[index]);
    if (arm_body_ids_[index] < 0)
    {
      RCLCPP_ERROR(logger_, "No body named '%s' exists in the MuJoCo model.", kArmBodyNames[index]);
      cleanup();
      return false;
    }
  }

  arm_attachment_site_id_ = mj_name2id(model_, mjOBJ_SITE, kArmAttachmentSiteName);
  if (arm_attachment_site_id_ < 0)
  {
    RCLCPP_ERROR(logger_, "No site named '%s' exists in the MuJoCo model.", kArmAttachmentSiteName);
    cleanup();
    return false;
  }

  obstacle_geom_id_ = mj_name2id(model_, mjOBJ_GEOM, obstacle_geom_name.c_str());
  if (obstacle_geom_id_ < 0 || model_->geom_type[obstacle_geom_id_] != mjGEOM_BOX)
  {
    RCLCPP_ERROR(logger_, "No box geom named '%s' exists in the MuJoCo model.", obstacle_geom_name.c_str());
    cleanup();
    return false;
  }

  get_robot_state_service_ = node_->create_service<GetRobotState>(
      kGetRobotStateService, std::bind(&SimulationStateProviderPlugin::handle_get_robot_state, this,
                                       std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(logger_, "SimulationStateProviderPlugin initialized. Service available at '%s'.",
              get_robot_state_service_->get_service_name());
  return true;
}

void SimulationStateProviderPlugin::update(const mjModel* /*model*/, mjData* /*data*/)
{
}

void SimulationStateProviderPlugin::cleanup()
{
  get_robot_state_service_.reset();
  model_ = nullptr;
  data_ = nullptr;
  nq_ = 0;
  arm_body_ids_.fill(-1);
  arm_attachment_site_id_ = -1;
  obstacle_geom_id_ = -1;
  node_.reset();
}

void SimulationStateProviderPlugin::handle_get_robot_state(const GetRobotState::Request::SharedPtr /*request*/,
                                                           GetRobotState::Response::SharedPtr response)
{
  auto* mutex = simulation_mutex();
  if (!mutex || !model_ || !data_ || arm_attachment_site_id_ < 0 || obstacle_geom_id_ < 0)
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

  response->arm_points_world.resize(arm_body_ids_.size() + 1);
  for (std::size_t index = 0; index < arm_body_ids_.size(); ++index)
  {
    const int body_id = arm_body_ids_[index];
    response->arm_points_world[index].x = static_cast<double>(data_->xpos[3 * body_id]);
    response->arm_points_world[index].y = static_cast<double>(data_->xpos[3 * body_id + 1]);
    response->arm_points_world[index].z = static_cast<double>(data_->xpos[3 * body_id + 2]);
  }
  auto& attachment_point = response->arm_points_world.back();
  attachment_point.x = static_cast<double>(data_->site_xpos[3 * arm_attachment_site_id_]);
  attachment_point.y = static_cast<double>(data_->site_xpos[3 * arm_attachment_site_id_ + 1]);
  attachment_point.z = static_cast<double>(data_->site_xpos[3 * arm_attachment_site_id_ + 2]);

  response->obstacle_position.x = static_cast<double>(data_->geom_xpos[3 * obstacle_geom_id_]);
  response->obstacle_position.y = static_cast<double>(data_->geom_xpos[3 * obstacle_geom_id_ + 1]);
  response->obstacle_position.z = static_cast<double>(data_->geom_xpos[3 * obstacle_geom_id_ + 2]);
  response->obstacle_size.x = 2.0 * static_cast<double>(model_->geom_size[3 * obstacle_geom_id_]);
  response->obstacle_size.y = 2.0 * static_cast<double>(model_->geom_size[3 * obstacle_geom_id_ + 1]);
  response->obstacle_size.z = 2.0 * static_cast<double>(model_->geom_size[3 * obstacle_geom_id_ + 2]);
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::SimulationStateProviderPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
