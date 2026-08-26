/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>

#include "mujoco_ros2_control/data.hpp"

namespace mujoco_ros2_control
{

/**
 * @brief Classify a MuJoCo actuator (motor / position / velocity / custom) from its bias type.
 */
ActuatorType get_actuator_type(const mjModel* mj_model, int mujoco_actuator_id);

/**
 * @brief Resolve which MuJoCo joint an actuator drives and record its addresses in mjData.
 *
 * Fills in the joint name, qpos/qvel addresses, actuator id and actuator type.
 * @return false when the actuator's transmission is unsupported or has no matching joint.
 */
bool map_actuator_to_joint(const mjModel* model, int actuator_id, MuJoCoActuatorData& actuator_data,
                           const rclcpp::Logger& logger);

/**
 * @brief Enable the command mode implied by the actuator's MuJoCo type.
 */
void initialize_actuator_control(MuJoCoActuatorData& actuator_data);

/**
 * @brief Register every MuJoCo joint that has no actuator driving it as a passive joint.
 */
void append_passive_actuators(const mjModel* model, std::vector<MuJoCoActuatorData>& actuators,
                              const rclcpp::Logger& logger);

/**
 * @brief Build the actuator data containers for every actuator in the MuJoCo model.
 *
 * One entry is created per MuJoCo actuator, mapped to the joint it drives and initialized with
 * the command mode its type implies. Every MuJoCo joint left without an actuator is then
 * appended as a passive actuator.
 *
 * @return false when any actuator could not be mapped to a joint.
 */
bool discover_mujoco_actuators(const mjModel* model, std::vector<MuJoCoActuatorData>& actuators,
                               const rclcpp::Logger& logger);

/**
 * @brief Find the non-passive actuator matching an actuator or joint name.
 * @return nullptr when no controllable actuator matches.
 */
MuJoCoActuatorData* find_controllable_actuator(std::vector<MuJoCoActuatorData>& actuators, const mjModel* model,
                                               const std::string& actuator_name);

}  // namespace mujoco_ros2_control
