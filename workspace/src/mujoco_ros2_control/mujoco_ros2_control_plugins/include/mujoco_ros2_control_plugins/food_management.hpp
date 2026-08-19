// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__FOOD_MANAGEMENT_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__FOOD_MANAGEMENT_HPP_

#include <string>

#include <mujoco/mujoco.h>

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Teleports free-floating food bodies into a parking area.
 *
 * Each food item is expected to be a body carrying a single free joint. The
 * manager rewrites that joint's qpos so the item appears at a requested pose,
 * expressed relative to a numbered parking body, at a fixed drop height.
 */
class FoodManagement
{
public:
  /**
   * @param model            The live MuJoCo model.
   * @param data             The live MuJoCo data.
   * @param throw_height     World-frame z (metres) the food is placed at.
   * @param parking_count    Number of parking bodies (indices 1..parking_count).
   * @param parking_prefix   Body-name prefix for parkings ("parking" -> "parking1").
   */
  FoodManagement(mjModel* model, mjData* data, double throw_height, int parking_count = 4,
                 std::string parking_prefix = "parking");

  bool is_available() const;

  /**
   * @brief Place a named food body inside a parking area.
   * @param parking_index 1-based parking index.
   * @param food_name     Name of the food body.
   * @param x             X in the parking frame.
   * @param y             Y in the parking frame.
   * @param quat          Orientation quaternion in MuJoCo order (w, x, y, z).
   * @param error         Populated with a human-readable reason on failure.
   * @return true on success.
   */
  bool throw_food(int parking_index, const std::string& food_name, double x, double y, const double quat[4],
                  std::string& error);

private:
  // Resolve the free joint of a named body; returns its id or -1 with a reason.
  int find_free_joint(const std::string& body_name, std::string& error) const;

  mjModel* model_;
  mjData* data_;
  double throw_height_;
  int parking_count_;
  std::string parking_prefix_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__FOOD_MANAGEMENT_HPP_
