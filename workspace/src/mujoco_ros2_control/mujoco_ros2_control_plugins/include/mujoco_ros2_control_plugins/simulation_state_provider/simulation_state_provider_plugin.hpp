// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__SIMULATION_STATE_PROVIDER_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__SIMULATION_STATE_PROVIDER_PLUGIN_HPP_

#include <array>
#include <cstddef>
#include <memory>

#include <mujoco_ros2_control_msgs/srv/get_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/** @brief Provides snapshots of the live MuJoCo state. */
class SimulationStateProviderPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  SimulationStateProviderPlugin() = default;
  ~SimulationStateProviderPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using GetRobotState = mujoco_ros2_control_msgs::srv::GetRobotState;

  void handle_get_robot_state(const GetRobotState::Request::SharedPtr request,
                              GetRobotState::Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("SimulationStateProviderPlugin") };
  rclcpp::Service<GetRobotState>::SharedPtr get_robot_state_service_;
  const mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  std::size_t nq_{ 0 };
  std::array<int, 6> arm_body_ids_{ { -1, -1, -1, -1, -1, -1 } };
  int arm_attachment_site_id_{ -1 };
  int obstacle_geom_id_{ -1 };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__SIMULATION_STATE_PROVIDER_PLUGIN_HPP_
