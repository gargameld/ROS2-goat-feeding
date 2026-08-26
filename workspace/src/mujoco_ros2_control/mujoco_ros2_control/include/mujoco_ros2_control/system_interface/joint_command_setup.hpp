/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>

#include "mujoco_ros2_control/data.hpp"
#include "mujoco_ros2_control/system_interface/component_info_map.hpp"

namespace mujoco_ros2_control
{

/**
 * @brief Seed a joint's state and command values from the URDF's `initial_value` attributes.
 */
void initialize_joint_interfaces(const hardware_interface::ComponentInfo& joint, URDFJointData& joint_data);

/**
 * @brief Return the joint's command interface names in position, velocity, effort order.
 */
std::vector<std::string> get_ordered_command_interfaces(const hardware_interface::ComponentInfo& joint);

/**
 * @brief Enable the actuator command modes the joint asks for, rejecting unsupported pairings.
 *
 * A MuJoCo position actuator can only serve a position command interface, a velocity actuator only
 * a velocity interface, and a motor/general actuator only an effort interface.
 *
 * @throws std::runtime_error when the joint declares command interfaces but none of them are
 *         supported by its actuator.
 */
void configure_joint_command_interfaces(const hardware_interface::ComponentInfo& joint,
                                        const std::string& actuator_name,
                                        const std::vector<std::string>& command_interface_names,
                                        MuJoCoActuatorData& actuator, const rclcpp::Logger& logger);

/**
 * @brief Build the URDF joint data containers described by the ros2_control hardware info.
 *
 * Each joint is matched to the MuJoCo actuator that drives it, seeded from the URDF's initial
 * values, and its command interfaces are configured on that actuator. A joint with command
 * interfaces but no matching actuator is demoted to a passive joint and exports none of them.
 *
 * @param[out] joints Joint data containers, resized to the hardware info's joints.
 * @param[out] joint_hardware_info Hardware info of every registered joint, keyed by joint name.
 */
void register_urdf_joints(const hardware_interface::HardwareInfo& hardware_info, const mjModel* model,
                          std::vector<MuJoCoActuatorData>& actuators, std::vector<URDFJointData>& joints,
                          ComponentInfoMap& joint_hardware_info, const rclcpp::Logger& logger);

}  // namespace mujoco_ros2_control
