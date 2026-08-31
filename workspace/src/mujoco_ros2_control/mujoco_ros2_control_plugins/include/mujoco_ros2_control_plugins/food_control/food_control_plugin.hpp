// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__FOOD_CONTROL_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__FOOD_CONTROL_PLUGIN_HPP_

#include <memory>

#include <mujoco_ros2_control_msgs/srv/throw_food.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"
#include "mujoco_ros2_control_plugins/food_control/food_management.hpp"

namespace mujoco_ros2_control_plugins
{

/** @brief Offers the service that teleports food into configured parking areas. */
class FoodControlPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  FoodControlPlugin() = default;
  ~FoodControlPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using ThrowFood = mujoco_ros2_control_msgs::srv::ThrowFood;

  void handle_throw_food(const ThrowFood::Request::SharedPtr request, ThrowFood::Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("FoodControlPlugin") };
  rclcpp::Service<ThrowFood>::SharedPtr throw_food_service_;
  std::unique_ptr<FoodManagement> food_management_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__FOOD_CONTROL_PLUGIN_HPP_
