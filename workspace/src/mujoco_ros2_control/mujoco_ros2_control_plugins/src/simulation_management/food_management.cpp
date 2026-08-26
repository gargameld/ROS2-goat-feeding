// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mujoco_ros2_control_plugins/simulation_management/food_management.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

namespace mujoco_ros2_control_plugins
{

FoodManagement::FoodManagement(mjModel* model, mjData* data, double throw_height, int parking_count,
                               std::string parking_prefix)
  : model_(model)
  , data_(data)
  , throw_height_(throw_height)
  , parking_count_(parking_count)
  , parking_prefix_(std::move(parking_prefix))
{
}

bool FoodManagement::is_available() const
{
  return model_ != nullptr && data_ != nullptr && parking_count_ > 0;
}

int FoodManagement::find_free_joint(const std::string& body_name, std::string& error) const
{
  const int body_id = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
  if (body_id < 0)
  {
    error = "No body named '" + body_name + "' exists in the simulation.";
    return -1;
  }

  const int first_joint = model_->body_jntadr[body_id];
  const int joint_count = model_->body_jntnum[body_id];
  for (int joint = first_joint; joint < first_joint + joint_count; ++joint)
  {
    if (model_->jnt_type[joint] == mjJNT_FREE)
    {
      return joint;
    }
  }

  error = "Body '" + body_name + "' has no free joint and cannot be thrown.";
  return -1;
}

bool FoodManagement::throw_food(int parking_index, const std::string& food_name, double x, double y,
                                const double quat[4], std::string& error)
{
  if (!is_available())
  {
    error = "The MuJoCo simulation is unavailable.";
    return false;
  }
  if (parking_index < 1 || parking_index > parking_count_)
  {
    error = "parking_index must be between 1 and " + std::to_string(parking_count_) + ".";
    return false;
  }

  const double planar[] = { x, y };
  if (!std::all_of(std::begin(planar), std::end(planar), [](double value) { return std::isfinite(value); }))
  {
    error = "The requested x/y position must be finite.";
    return false;
  }
  if (!std::all_of(quat, quat + 4, [](double value) { return std::isfinite(value); }))
  {
    error = "The orientation quaternion must contain finite values.";
    return false;
  }

  // Normalise the requested orientation so a lazily specified quaternion is valid.
  double request_quat[4] = { quat[0], quat[1], quat[2], quat[3] };
  const double quat_norm = std::sqrt(request_quat[0] * request_quat[0] + request_quat[1] * request_quat[1] +
                                     request_quat[2] * request_quat[2] + request_quat[3] * request_quat[3]);
  if (quat_norm < 1e-9)
  {
    error = "The orientation quaternion must have a non-zero magnitude.";
    return false;
  }
  for (double& component : request_quat)
  {
    component /= quat_norm;
  }

  const std::string parking_name = parking_prefix_ + std::to_string(parking_index);
  const int parking_id = mj_name2id(model_, mjOBJ_BODY, parking_name.c_str());
  if (parking_id < 0)
  {
    error = "No parking body named '" + parking_name + "' exists in the simulation.";
    return false;
  }

  const int joint = find_free_joint(food_name, error);
  if (joint < 0)
  {
    return false;
  }

  // Compose the parking-frame request into the world frame using the parking's
  // current world pose (position, orientation) from the forward kinematics.
  mjtNum parking_quat[4];
  mju_copy4(parking_quat, data_->xquat + 4 * parking_id);

  const mjtNum local_offset[3] = { x, y, 0.0 };
  mjtNum world_offset[3];
  mju_rotVecQuat(world_offset, local_offset, parking_quat);

  mjtNum world_pos[3];
  mju_add3(world_pos, data_->xpos + 3 * parking_id, world_offset);

  const mjtNum req_quat[4] = { request_quat[0], request_quat[1], request_quat[2], request_quat[3] };
  mjtNum world_quat[4];
  mju_mulQuat(world_quat, parking_quat, req_quat);

  const int qpos_adr = model_->jnt_qposadr[joint];
  const int dof_adr = model_->jnt_dofadr[joint];

  data_->qpos[qpos_adr + 0] = world_pos[0];
  data_->qpos[qpos_adr + 1] = world_pos[1];
  data_->qpos[qpos_adr + 2] = throw_height_;  // constant world-frame drop height
  data_->qpos[qpos_adr + 3] = world_quat[0];
  data_->qpos[qpos_adr + 4] = world_quat[1];
  data_->qpos[qpos_adr + 5] = world_quat[2];
  data_->qpos[qpos_adr + 6] = world_quat[3];

  // Clear the free joint's linear and angular velocity so the item starts at rest.
  for (int dof = 0; dof < 6; ++dof)
  {
    data_->qvel[dof_adr + dof] = 0.0;
  }

  mj_forward(model_, data_);
  return true;
}

}  // namespace mujoco_ros2_control_plugins
