// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__OBSTACLE_MANAGEMENT_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__OBSTACLE_MANAGEMENT_HPP_

#include <string>

#include <mujoco/mujoco.h>

namespace mujoco_ros2_control_plugins
{

struct ObstacleState
{
  double x;
  double y;
  double z;
  double width;
  double length;
  double height;
};

/**
 * @brief Edits a named static box through mjSpec and recompiles the live simulation.
 */
class ObstacleManagement
{
public:
  ObstacleManagement(mjSpec* spec, mjModel* model, mjData* data, std::string geom_name = "obstacle");

  bool is_available() const;
  ObstacleState state() const;
  bool set_obstacle(double x, double y, double width, double length, double height, std::string& error);

private:
  mjsGeom* find_geom() const;

  mjSpec* spec_;
  mjModel* model_;
  mjData* data_;
  std::string geom_name_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__OBSTACLE_MANAGEMENT_HPP_
