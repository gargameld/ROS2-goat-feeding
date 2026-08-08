/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <transmission_interface/transmission.hpp>

#include "mujoco_ros2_control/data.hpp"

namespace mujoco_ros2_control::detail
{

bool populate_actuator_model_data(const mjModel* model, int actuator_id, MuJoCoActuatorData& actuator_data,
                                  const rclcpp::Logger& logger);

void initialize_actuator_control(MuJoCoActuatorData& actuator_data, const rclcpp::Node::SharedPtr& node);

void append_passive_actuators(const mjModel* model, const hardware_interface::HardwareInfo& hardware_info,
                              std::vector<MuJoCoActuatorData>& actuators, const rclcpp::Logger& logger);

void initialize_actuator_states(const mjData* data, std::vector<MuJoCoActuatorData>& actuators);

void configure_mimic_joint(hardware_interface::ComponentInfo& joint,
                           const hardware_interface::HardwareInfo& hardware_info, URDFJointData& joint_data,
                           const rclcpp::Logger& logger);

MuJoCoActuatorData* find_controllable_actuator(std::vector<MuJoCoActuatorData>& actuators, const mjModel* model,
                                               const std::string& actuator_name);

void initialize_joint_interfaces(const hardware_interface::ComponentInfo& joint, URDFJointData& joint_data);

std::vector<std::string> get_ordered_command_interfaces(const hardware_interface::ComponentInfo& joint);

void configure_position_command_interface(const std::string& actuator_name, MuJoCoActuatorData& actuator,
                                          const rclcpp::Logger& logger);

void configure_velocity_command_interface(const std::string& actuator_name, MuJoCoActuatorData& actuator,
                                          const rclcpp::Logger& logger);

void configure_effort_command_interface(const std::string& actuator_name, MuJoCoActuatorData& actuator,
                                        const rclcpp::Logger& logger);

void configure_joint_command_interfaces(const hardware_interface::ComponentInfo& joint,
                                        const std::string& actuator_name,
                                        const std::vector<std::string>& command_interface_names,
                                        MuJoCoActuatorData& actuator, const rclcpp::Logger& logger);

bool transmission_actuators_exist(const hardware_interface::TransmissionInfo& transmission, const mjModel* model,
                                  const rclcpp::Logger& logger);

bool transmission_joint_interfaces_match(const hardware_interface::TransmissionInfo& transmission,
                                         const rclcpp::Logger& logger);

bool make_transmission_joint_handles(const hardware_interface::TransmissionInfo& transmission,
                                     std::vector<URDFJointData>& joints,
                                     std::vector<transmission_interface::JointHandle>& handles,
                                     const rclcpp::Logger& logger);

bool make_transmission_actuator_handles(const hardware_interface::TransmissionInfo& transmission,
                                        std::vector<MuJoCoActuatorData>& actuators,
                                        std::vector<transmission_interface::ActuatorHandle>& handles,
                                        const rclcpp::Logger& logger);

std::optional<FTSensorData> make_force_torque_sensor(const hardware_interface::ComponentInfo& sensor,
                                                     const std::string& mujoco_sensor_name,
                                                     const hardware_interface::HardwareInfo& hardware_info,
                                                     const mjModel* model, const rclcpp::Logger& logger);

std::optional<IMUSensorData> make_imu_sensor(const hardware_interface::ComponentInfo& sensor,
                                             const std::string& mujoco_sensor_name,
                                             const hardware_interface::HardwareInfo& hardware_info,
                                             const mjModel* model, const rclcpp::Logger& logger);

}  // namespace mujoco_ros2_control::detail
