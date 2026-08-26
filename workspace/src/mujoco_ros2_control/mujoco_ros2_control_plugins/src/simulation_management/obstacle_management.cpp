// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mujoco_ros2_control_plugins/simulation_management/obstacle_management.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace mujoco_ros2_control_plugins
{

ObstacleManagement::ObstacleManagement(mjSpec* spec, mjModel* model, mjData* data, std::string geom_name)
  : spec_(spec), model_(model), data_(data), geom_name_(std::move(geom_name))
{
}

bool ObstacleManagement::is_available() const
{
  if (!spec_ || !model_ || !data_ || !find_geom())
  {
    return false;
  }
  const int geom_id = mj_name2id(model_, mjOBJ_GEOM, geom_name_.c_str());
  return geom_id >= 0 && model_->geom_type[geom_id] == mjGEOM_BOX;
}

ObstacleState ObstacleManagement::state() const
{
  const int geom_id = model_ ? mj_name2id(model_, mjOBJ_GEOM, geom_name_.c_str()) : -1;
  if (geom_id < 0 || model_->geom_type[geom_id] != mjGEOM_BOX)
  {
    throw std::runtime_error("The managed obstacle box is unavailable.");
  }

  return ObstacleState{
    static_cast<double>(model_->geom_pos[3 * geom_id]),
    static_cast<double>(model_->geom_pos[3 * geom_id + 1]),
    static_cast<double>(model_->geom_pos[3 * geom_id + 2]),
    2.0 * static_cast<double>(model_->geom_size[3 * geom_id]),
    2.0 * static_cast<double>(model_->geom_size[3 * geom_id + 1]),
    2.0 * static_cast<double>(model_->geom_size[3 * geom_id + 2]),
  };
}

bool ObstacleManagement::set_obstacle(double x, double y, double width, double length, double height,
                                      std::string& error)
{
  const double values[] = { x, y, width, length, height };
  if (!std::all_of(std::begin(values), std::end(values), [](double value) { return std::isfinite(value); }))
  {
    error = "Obstacle position and dimensions must be finite.";
    return false;
  }
  if (width <= 0.0 || length <= 0.0 || height <= 0.0)
  {
    error = "Obstacle dimensions must be greater than zero.";
    return false;
  }

  mjsGeom* geom = find_geom();
  if (!geom || geom->type != mjGEOM_BOX)
  {
    error = "Could not find the named box geom '" + geom_name_ + "' in the MuJoCo specification.";
    return false;
  }

  // Pull runtime-adjustable values back into the spec before applying this edit.
  if (!mj_copyBack(spec_, model_))
  {
    error = "MuJoCo could not copy the live model values back into its specification.";
    return false;
  }

  const double old_position[] = { geom->pos[0], geom->pos[1], geom->pos[2] };
  const double old_size[] = { geom->size[0], geom->size[1], geom->size[2] };
  geom->pos[0] = x;
  geom->pos[1] = y;
  geom->pos[2] = height / 2.0;
  geom->size[0] = width / 2.0;
  geom->size[1] = length / 2.0;
  geom->size[2] = height / 2.0;

  if (mj_recompile(spec_, nullptr, model_, data_) != 0)
  {
    const char* compile_error = mjs_getError(spec_);
    error = compile_error && compile_error[0] ? compile_error : "MuJoCo failed to recompile the obstacle edit.";
    std::copy(std::begin(old_position), std::end(old_position), geom->pos);
    std::copy(std::begin(old_size), std::end(old_size), geom->size);
    (void)mj_recompile(spec_, nullptr, model_, data_);
    return false;
  }

  mj_forward(model_, data_);
  return true;
}

mjsGeom* ObstacleManagement::find_geom() const
{
  if (!spec_)
  {
    return nullptr;
  }
  return mjs_asGeom(mjs_findElement(spec_, mjOBJ_GEOM, geom_name_.c_str()));
}

}  // namespace mujoco_ros2_control_plugins
