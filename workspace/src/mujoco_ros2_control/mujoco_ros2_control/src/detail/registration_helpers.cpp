/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/detail/registration_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/version.h>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control/detail/configuration_helpers.hpp"
#include "mujoco_ros2_control/detail/model_mapping_helpers.hpp"

namespace mujoco_ros2_control::detail
{

bool populate_actuator_model_data(const mjModel* model, int actuator_id, MuJoCoActuatorData& actuator_data,
                                  const rclcpp::Logger& logger)
{
  int transmission_type = model->actuator_trntype[actuator_id];
  int target_id = model->actuator_trnid[actuator_id * 2];
  const char* actuator_name = mj_id2name(model, mjOBJ_ACTUATOR, actuator_id);
  if (!actuator_name)
  {
    actuator_name = "unnamed";
  }

  if (transmission_type == mjTRN_JOINT)
  {
    const char* joint_name = mj_id2name(model, mjOBJ_JOINT, target_id);
    if (joint_name)
    {
      actuator_data.joint_name = std::string(joint_name);
      RCLCPP_INFO(logger, "Registering MuJoCo actuator '%s' for joint '%s'", actuator_data.joint_name.c_str(),
                  joint_name);
    }
    else
    {
      RCLCPP_ERROR(logger, "Failed to find joint name for actuator '%s'", actuator_name);
      return false;
    }
  }
  else if (transmission_type == mjTRN_TENDON)
  {
    int joint_id = mj_name2id(model, mjOBJ_JOINT, actuator_name);
    if (joint_id != -1)
    {
      target_id = joint_id;
      actuator_data.joint_name = std::string(actuator_name);
      RCLCPP_INFO(logger, "Registering MuJoCo tendon actuator '%s' using joint state", actuator_name);
    }
    else
    {
      RCLCPP_ERROR(logger,
                   "Tendon actuator '%s' has no matching joint. Tendon actuators must be named the same as a joint "
                   "that they will control.",
                   actuator_name);
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Unsupported transmission type '%d' for actuator '%s'", transmission_type, actuator_name);
    return false;
  }

  actuator_data.mj_actuator_id = actuator_id;
  actuator_data.mj_pos_adr = model->jnt_qposadr[target_id];
  actuator_data.mj_vel_adr = model->jnt_dofadr[target_id];
  actuator_data.mj_joint_type = model->jnt_type[target_id];
  actuator_data.actuator_type = get_actuator_type(model, actuator_data.mj_actuator_id);
  return true;
}

void initialize_actuator_control(MuJoCoActuatorData& actuator_data, const rclcpp::Node::SharedPtr& node)
{
  const auto initialize_position_pid = [&]() -> bool {
#if HARDWARE_INTERFACE_VERSION_MAJOR < 3
    actuator_data.pos_pid =
        std::make_shared<control_toolbox::PidROS>(node, "pid_gains.position." + actuator_data.joint_name, false);
    actuator_data.pos_pid->initPid();
    const auto gains = actuator_data.pos_pid->getGains();
#else
    actuator_data.pos_pid =
        std::make_shared<control_toolbox::PidROS>(node, "pid_gains.position." + actuator_data.joint_name, "", false);
    actuator_data.pos_pid->initialize_from_ros_parameters();
    const auto gains = actuator_data.pos_pid->get_gains();
#endif
    return std::isfinite(gains.p_gain_) && std::isfinite(gains.i_gain_) && std::isfinite(gains.d_gain_);
  };

  const auto initialize_velocity_pid = [&]() -> bool {
#if HARDWARE_INTERFACE_VERSION_MAJOR < 3
    actuator_data.vel_pid =
        std::make_shared<control_toolbox::PidROS>(node, "pid_gains.velocity." + actuator_data.joint_name, false);
    actuator_data.vel_pid->initPid();
    const auto gains = actuator_data.pos_pid->getGains();
#else
    actuator_data.vel_pid =
        std::make_shared<control_toolbox::PidROS>(node, "pid_gains.velocity." + actuator_data.joint_name, "", false);
    actuator_data.vel_pid->initialize_from_ros_parameters();
    const auto gains = actuator_data.vel_pid->get_gains();
#endif
    return std::isfinite(gains.p_gain_) && std::isfinite(gains.i_gain_) && std::isfinite(gains.d_gain_);
  };

  if (actuator_data.actuator_type == ActuatorType::POSITION)
  {
    actuator_data.is_position_control_enabled = true;
  }
  else if (actuator_data.actuator_type == ActuatorType::VELOCITY)
  {
    actuator_data.has_pos_pid = initialize_position_pid();
    actuator_data.is_velocity_control_enabled = true;
  }
  else if (actuator_data.actuator_type == ActuatorType::MOTOR || actuator_data.actuator_type == ActuatorType::CUSTOM)
  {
    actuator_data.has_pos_pid = initialize_position_pid();
    actuator_data.has_vel_pid = initialize_velocity_pid();
    actuator_data.is_effort_control_enabled = true;
  }
}

void append_passive_actuators(const mjModel* model, const hardware_interface::HardwareInfo& hardware_info,
                              std::vector<MuJoCoActuatorData>& actuators, const rclcpp::Logger& logger)
{
  for (int joint_id = 0; joint_id < model->njnt; joint_id++)
  {
    const auto actuator_it =
        std::find_if(actuators.cbegin(), actuators.cend(), [model, joint_id](const MuJoCoActuatorData& actuator) {
          return actuator.mj_pos_adr == model->jnt_qposadr[joint_id];
        });
    if (actuator_it == actuators.cend() && model->jnt_type[joint_id] != mjJNT_FREE &&
        model->jnt_type[joint_id] != mjJNT_BALL)
    {
      MuJoCoActuatorData passive_actuator;
      passive_actuator.joint_name = std::string(mj_id2name(model, mjOBJ_JOINT, joint_id));
      const bool is_mimic = is_mimic_joint(passive_actuator.joint_name, hardware_info);
      RCLCPP_INFO_EXPRESSION(logger, !is_mimic,
                             "MuJoCo joint '%s' has no associated actuator. Registering as a passive joint.",
                             passive_actuator.joint_name.c_str());
      RCLCPP_INFO_EXPRESSION(logger, is_mimic, "MuJoCo joint '%s' is a mimic joint and has no associated actuator.",
                             passive_actuator.joint_name.c_str());
      passive_actuator.mj_pos_adr = model->jnt_qposadr[joint_id];
      passive_actuator.mj_vel_adr = model->jnt_dofadr[joint_id];
      passive_actuator.mj_joint_type = model->jnt_type[joint_id];
      passive_actuator.actuator_type = ActuatorType::PASSIVE;
      actuators.push_back(passive_actuator);
    }
  }
}

void initialize_actuator_states(const mjData* data, std::vector<MuJoCoActuatorData>& actuators)
{
  for (auto& actuator_data : actuators)
  {
    actuator_data.position_interface.state_ = data->qpos[actuator_data.mj_pos_adr];
    actuator_data.velocity_interface.state_ = data->qvel[actuator_data.mj_vel_adr];
    actuator_data.effort_interface.state_ = 0.0;

    if (actuator_data.actuator_type != ActuatorType::PASSIVE)
    {
      actuator_data.position_interface.command_ = actuator_data.position_interface.state_;
      actuator_data.velocity_interface.command_ = actuator_data.velocity_interface.state_;
      actuator_data.effort_interface.command_ = actuator_data.effort_interface.state_;
    }
  }
}

void configure_mimic_joint(hardware_interface::ComponentInfo& joint,
                           const hardware_interface::HardwareInfo& hardware_info, URDFJointData& joint_data,
                           const rclcpp::Logger& logger)
{
  if (joint.parameters.find("mimic") == joint.parameters.end())
  {
    return;
  }

  const auto mimicked_joint = joint.parameters.at("mimic");
  const auto mimicked_joint_it =
      std::find_if(hardware_info.joints.begin(), hardware_info.joints.end(),
                   [&mimicked_joint](const hardware_interface::ComponentInfo& info) {
                     return info.name == mimicked_joint;
                   });
  if (mimicked_joint_it == hardware_info.joints.end())
  {
    throw std::runtime_error(std::string("Mimicked joint '") + mimicked_joint + "' not found");
  }
  joint_data.is_mimic = true;
  joint_data.mimicked_joint_index = std::distance(hardware_info.joints.begin(), mimicked_joint_it);

  auto parameter_it = joint.parameters.find("multiplier");
  if (parameter_it != joint.parameters.end())
  {
    joint_data.mimic_multiplier = std::stod(joint.parameters.at("multiplier"));
  }
  else
  {
    joint_data.mimic_multiplier = 1.0;
  }
  RCLCPP_INFO_EXPRESSION(logger, joint_data.is_mimic,
                         "Joint : '%s' is a mimic joint mimicking joint '%s' with multiplier '%.2f'",
                         joint.name.c_str(), hardware_info.joints.at(joint_data.mimicked_joint_index).name.c_str(),
                         joint_data.mimic_multiplier);
  if (!joint.command_interfaces.empty())
  {
    RCLCPP_ERROR(logger,
                 "Joint : '%s' is a mimic joint but has command interfaces defined. Command interfaces for mimic joints "
                 "will be ignored and this joint will be only mimicked.",
                 joint.name.c_str());
    joint.command_interfaces.clear();
  }
}

MuJoCoActuatorData* find_controllable_actuator(std::vector<MuJoCoActuatorData>& actuators, const mjModel* model,
                                               const std::string& actuator_name)
{
  const auto actuator_it =
      std::find_if(actuators.begin(), actuators.end(), [&actuator_name, model](const MuJoCoActuatorData& actuator) {
        return (actuator.actuator_type != ActuatorType::PASSIVE) &&
               ((mj_id2name(model, mjOBJ_ACTUATOR, actuator.mj_actuator_id) == actuator_name) ||
                (actuator.joint_name == actuator_name));
      });
  return actuator_it == actuators.end() ? nullptr : &*actuator_it;
}

void initialize_joint_interfaces(const hardware_interface::ComponentInfo& joint, URDFJointData& joint_data)
{
  auto get_initial_value = [](const hardware_interface::InterfaceInfo& interface_info) {
    if (!interface_info.initial_value.empty())
    {
      double value = std::stod(interface_info.initial_value);
      return value;
    }
    return 0.0;
  };

  for (const auto& state_if : joint.state_interfaces)
  {
    if (state_if.name == hardware_interface::HW_IF_POSITION)
    {
      joint_data.position_interface.state_ = get_initial_value(state_if);
    }
    else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
    {
      joint_data.velocity_interface.state_ = get_initial_value(state_if);
    }
    else if (state_if.name == hardware_interface::HW_IF_EFFORT || state_if.name == hardware_interface::HW_IF_TORQUE ||
             state_if.name == hardware_interface::HW_IF_FORCE)
    {
      joint_data.effort_interface.state_ = get_initial_value(state_if);
    }

    joint_data.position_interface.command_ = joint_data.position_interface.state_;
    joint_data.velocity_interface.command_ = joint_data.velocity_interface.state_;
    joint_data.effort_interface.command_ = joint_data.effort_interface.state_;
  }
}

std::vector<std::string> get_ordered_command_interfaces(const hardware_interface::ComponentInfo& joint)
{
  std::vector<std::string> joint_command_interfaces;
  std::transform(joint.command_interfaces.begin(), joint.command_interfaces.end(),
                 std::back_inserter(joint_command_interfaces),
                 [](const hardware_interface::InterfaceInfo& interface_info) { return interface_info.name; });
  return get_interfaces_in_order(joint_command_interfaces,
                                 { hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
                                   hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_TORQUE,
                                   hardware_interface::HW_IF_FORCE });
}

void configure_position_command_interface(const std::string& actuator_name, MuJoCoActuatorData& actuator,
                                          const rclcpp::Logger& logger)
{
  if (actuator.actuator_type == ActuatorType::POSITION)
  {
    RCLCPP_INFO(logger, "Using MuJoCo position actuator for the joint : '%s'", actuator_name.c_str());
    actuator.is_position_control_enabled = true;
  }
  else if (actuator.actuator_type == ActuatorType::VELOCITY || actuator.actuator_type == ActuatorType::MOTOR ||
           actuator.actuator_type == ActuatorType::CUSTOM)
  {
    if (actuator.has_pos_pid)
    {
      actuator.is_position_control_enabled = false;
      actuator.is_position_pid_control_enabled = true;
#if HARDWARE_INTERFACE_VERSION_MAJOR >= 3
      const auto gains = actuator.pos_pid->get_gains();
      RCLCPP_INFO(logger,
                  "Position control PID gains for joint %s : P=%.4f, I=%.4f, D=%.4f, Imax=%.4f, Imin=%.4f, "
                  "Umin=%.4f, Umax=%.4f, antiwindup_strategy=%s",
                  actuator_name.c_str(), gains.p_gain_, gains.i_gain_, gains.d_gain_, gains.antiwindup_strat_.i_max,
                  gains.antiwindup_strat_.i_min, gains.u_min_, gains.u_max_,
                  gains.antiwindup_strat_.to_string().c_str());
#endif
    }
    else
    {
      RCLCPP_ERROR(logger,
                   "Position command interface for the joint : %s is not supported with velocity or motor "
                   "actuator without defining the PIDs",
                   actuator_name.c_str());
    }
  }
}

void configure_velocity_command_interface(const std::string& actuator_name, MuJoCoActuatorData& actuator,
                                          const rclcpp::Logger& logger)
{
  RCLCPP_ERROR_EXPRESSION(logger, actuator.actuator_type == ActuatorType::POSITION,
                          "Velocity command interface for the joint : %s is not supported with position actuator",
                          actuator_name.c_str());
  if (actuator.actuator_type == ActuatorType::VELOCITY)
  {
    RCLCPP_INFO(logger, "Using MuJoCo velocity actuator for the joint : '%s'", actuator_name.c_str());
    actuator.is_velocity_control_enabled = true;
  }
  else if (actuator.actuator_type == ActuatorType::MOTOR || actuator.actuator_type == ActuatorType::CUSTOM)
  {
    if (actuator.has_vel_pid)
    {
      actuator.is_velocity_control_enabled = false;
      actuator.is_velocity_pid_control_enabled = true;
#if HARDWARE_INTERFACE_VERSION_MAJOR >= 3
      const auto gains = actuator.vel_pid->get_gains();
      RCLCPP_INFO(logger,
                  "Velocity control PID gains for joint %s : P=%.4f, I=%.4f, D=%.4f, Imax=%.4f, Imin=%.4f, "
                  "Umin=%.4f, Umax=%.4f, antiwindup_strategy=%s",
                  actuator_name.c_str(), gains.p_gain_, gains.i_gain_, gains.d_gain_, gains.antiwindup_strat_.i_max,
                  gains.antiwindup_strat_.i_min, gains.u_min_, gains.u_max_,
                  gains.antiwindup_strat_.to_string().c_str());
#endif
    }
    else
    {
      RCLCPP_ERROR(logger,
                   "Velocity command interface for the joint : %s is not supported with motor or custom actuator "
                   "without defining the PIDs",
                   actuator_name.c_str());
    }
  }
}

void configure_effort_command_interface(const std::string& actuator_name, MuJoCoActuatorData& actuator,
                                        const rclcpp::Logger& logger)
{
  RCLCPP_ERROR_EXPRESSION(
      logger, actuator.actuator_type == ActuatorType::POSITION || actuator.actuator_type == ActuatorType::VELOCITY,
      "Effort command interface for the joint : %s is not supported with position or velocity actuator."
      "Skipping it.",
      actuator_name.c_str());
  if (actuator.actuator_type == ActuatorType::MOTOR || actuator.actuator_type == ActuatorType::CUSTOM)
  {
    RCLCPP_INFO(logger, "Using MuJoCo motor or custom actuator for the joint : '%s'", actuator_name.c_str());
    actuator.is_effort_control_enabled = true;
  }
}

void configure_joint_command_interfaces(const hardware_interface::ComponentInfo& joint,
                                        const std::string& actuator_name,
                                        const std::vector<std::string>& command_interface_names,
                                        MuJoCoActuatorData& actuator, const rclcpp::Logger& logger)
{
  for (const auto& command_if : command_interface_names)
  {
    if (command_if == hardware_interface::HW_IF_POSITION)
    {
      configure_position_command_interface(actuator_name, actuator, logger);
    }
    else if (command_if == hardware_interface::HW_IF_VELOCITY)
    {
      configure_velocity_command_interface(actuator_name, actuator, logger);
    }
    else if (command_if == hardware_interface::HW_IF_EFFORT || command_if == hardware_interface::HW_IF_TORQUE ||
             command_if == hardware_interface::HW_IF_FORCE)
    {
      configure_effort_command_interface(actuator_name, actuator, logger);
    }
    else
    {
      RCLCPP_WARN(logger, "Unsupported command interface '%s' for joint '%s'. Skipping it!", command_if.c_str(),
                  joint.name.c_str());
    }
  }

  if (!command_interface_names.empty() && !actuator.is_position_control_enabled &&
      !actuator.is_velocity_control_enabled && !actuator.is_effort_control_enabled &&
      !actuator.is_position_pid_control_enabled && !actuator.is_velocity_pid_control_enabled)
  {
    throw std::runtime_error("Joint '" + joint.name + "' which uses actuator '" + actuator_name +
                             "' has an unsupported command interface for the specified MuJoCo actuator");
  }
}

bool transmission_actuators_exist(const hardware_interface::TransmissionInfo& transmission, const mjModel* model,
                                  const rclcpp::Logger& logger)
{
  bool all_transmission_actuators = true;
  for (const auto& actuator : transmission.actuators)
  {
    RCLCPP_DEBUG(logger, "Actuator name: %s", actuator.name.c_str());
    if (get_actuator_id(actuator.name, model) != -1)
    {
      RCLCPP_INFO(logger, "Transmission actuator '%s' matches the MuJoCo actuator", actuator.name.c_str());
      all_transmission_actuators &= true;
    }
    else
    {
      RCLCPP_WARN(logger, "Transmission actuator '%s' not found in MuJoCo model", actuator.name.c_str());
      all_transmission_actuators &= false;
    }
  }
  return all_transmission_actuators;
}

bool transmission_joint_interfaces_match(const hardware_interface::TransmissionInfo& transmission,
                                         const rclcpp::Logger& logger)
{
  const auto first_joint_command_interfaces = transmission.joints.front().command_interfaces;
  for (const auto& joint : transmission.joints)
  {
    const auto joint_hardware_types = joint.command_interfaces;
    bool is_same =
        (joint_hardware_types.size() == first_joint_command_interfaces.size()) &&
        std::all_of(joint_hardware_types.begin(), joint_hardware_types.end(), [&](const std::string& hardware_type) {
          return std::find(first_joint_command_interfaces.begin(), first_joint_command_interfaces.end(),
                           hardware_type) != first_joint_command_interfaces.end();
        });
    if (!is_same)
    {
      RCLCPP_FATAL(logger,
                   "All joints of transmission '%s' must have the same command interfaces. Joint '%s' has different "
                   "interfaces than joint '%s'.",
                   transmission.name.c_str(), joint.name.c_str(), transmission.joints.front().name.c_str());
      return false;
    }
  }
  return true;
}

namespace
{

double* interface_passthrough(InterfaceData& position, InterfaceData& velocity, InterfaceData& effort,
                              const std::string& interface_name)
{
  if (interface_name == hardware_interface::HW_IF_POSITION)
  {
    return &position.transmission_passthrough_;
  }
  if (interface_name == hardware_interface::HW_IF_VELOCITY)
  {
    return &velocity.transmission_passthrough_;
  }
  if (interface_name == hardware_interface::HW_IF_EFFORT || interface_name == hardware_interface::HW_IF_TORQUE ||
      interface_name == hardware_interface::HW_IF_FORCE)
  {
    return &effort.transmission_passthrough_;
  }
  return nullptr;
}

}  // namespace

bool make_transmission_joint_handles(const hardware_interface::TransmissionInfo& transmission,
                                     std::vector<URDFJointData>& joints,
                                     std::vector<transmission_interface::JointHandle>& handles,
                                     const rclcpp::Logger& logger)
{
  for (const auto& joint_info : transmission.joints)
  {
    RCLCPP_INFO(logger, "\tCreating joint handle for joint: %s", joint_info.name.c_str());
    std::vector<std::string> hardware_types = joint_info.state_interfaces;
    add_items(hardware_types, joint_info.command_interfaces);

    auto joint_it = std::find_if(joints.begin(), joints.end(), [&](const auto& joint) {
      return joint.name == joint_info.name;
    });
    if (joint_it == joints.end())
    {
      RCLCPP_FATAL(logger, "Joint '%s' not found in the URDF joint data", joint_info.name.c_str());
      return false;
    }

    for (const auto& hardware_interface : hardware_types)
    {
      double* passthrough = interface_passthrough(joint_it->position_interface, joint_it->velocity_interface,
                                                  joint_it->effort_interface, hardware_interface);
      transmission_interface::JointHandle joint_handle(joint_info.name, hardware_interface, passthrough);
      handles.push_back(joint_handle);
    }
  }
  return true;
}

bool make_transmission_actuator_handles(const hardware_interface::TransmissionInfo& transmission,
                                        std::vector<MuJoCoActuatorData>& actuators,
                                        std::vector<transmission_interface::ActuatorHandle>& handles,
                                        const rclcpp::Logger& logger)
{
  for (const auto& actuator_info : transmission.actuators)
  {
    RCLCPP_INFO(logger, "\tCreating actuator handle for actuator: %s", actuator_info.name.c_str());
    const std::vector<std::string> hardware_types = {
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT,
      hardware_interface::HW_IF_TORQUE, hardware_interface::HW_IF_FORCE
    };
    auto actuator_it = std::find_if(actuators.begin(), actuators.end(),
                                    [&](const auto& actuator) { return actuator.joint_name == actuator_info.name; });
    if (actuator_it == actuators.end())
    {
      RCLCPP_FATAL(logger, "Actuator '%s' not found in the MuJoCo actuator data", actuator_info.name.c_str());
      return false;
    }
    if (actuator_it->actuator_type == ActuatorType::PASSIVE)
    {
      RCLCPP_FATAL(logger, "Actuator '%s' is passive and cannot be used in a transmission", actuator_info.name.c_str());
      return false;
    }
    for (const auto& hardware_interface : hardware_types)
    {
      double* passthrough = interface_passthrough(actuator_it->position_interface, actuator_it->velocity_interface,
                                                  actuator_it->effort_interface, hardware_interface);
      transmission_interface::ActuatorHandle actuator_handle(actuator_info.name, hardware_interface, passthrough);
      handles.push_back(actuator_handle);
    }
  }
  return true;
}

std::optional<FTSensorData> make_force_torque_sensor(const hardware_interface::ComponentInfo& sensor,
                                                     const std::string& mujoco_sensor_name,
                                                     const hardware_interface::HardwareInfo& hardware_info,
                                                     const mjModel* model, const rclcpp::Logger& logger)
{
  FTSensorData sensor_data;
  sensor_data.name = sensor.name;
  sensor_data.force.name =
      mujoco_sensor_name + get_hardware_parameter_or(hardware_info, "force_mjcf_suffix", "_force");
  sensor_data.torque.name =
      mujoco_sensor_name + get_hardware_parameter_or(hardware_info, "torque_mjcf_suffix", "_torque");

  int force_sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_data.force.name.c_str());
  int torque_sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_data.torque.name.c_str());
  if (force_sensor_id == -1 || torque_sensor_id == -1)
  {
    RCLCPP_ERROR(logger, "Failed to find force/torque sensor in MuJoCo model, sensor name: '%s'", sensor.name.c_str());
    return std::nullopt;
  }

  sensor_data.force.mj_sensor_index = model->sensor_adr[force_sensor_id];
  sensor_data.torque.mj_sensor_index = model->sensor_adr[torque_sensor_id];
  return sensor_data;
}

std::optional<IMUSensorData> make_imu_sensor(const hardware_interface::ComponentInfo& sensor,
                                             const std::string& mujoco_sensor_name,
                                             const hardware_interface::HardwareInfo& hardware_info,
                                             const mjModel* model, const rclcpp::Logger& logger)
{
  IMUSensorData sensor_data;
  sensor_data.name = sensor.name;
  sensor_data.orientation.name =
      mujoco_sensor_name + get_hardware_parameter_or(hardware_info, "orientation_mjcf_suffix", "_quat");
  sensor_data.angular_velocity.name =
      mujoco_sensor_name + get_hardware_parameter_or(hardware_info, "angular_velocity_mjcf_suffix", "_gyro");
  sensor_data.linear_acceleration.name =
      mujoco_sensor_name + get_hardware_parameter_or(hardware_info, "linear_acceleration_mjcf_suffix", "_accel");

  sensor_data.orientation_covariance.resize(9, 0.0);
  sensor_data.angular_velocity_covariance.resize(9, 0.0);
  sensor_data.linear_acceleration_covariance.resize(9, 0.0);

  const int quaternion_id = mj_name2id(model, mjOBJ_SENSOR, sensor_data.orientation.name.c_str());
  const int gyroscope_id = mj_name2id(model, mjOBJ_SENSOR, sensor_data.angular_velocity.name.c_str());
  const int accelerometer_id = mj_name2id(model, mjOBJ_SENSOR, sensor_data.linear_acceleration.name.c_str());

  RCLCPP_ERROR_EXPRESSION(logger, quaternion_id == -1, "Failed to find IMU sensor '%s' in MuJoCo model",
                          sensor_data.orientation.name.c_str());
  RCLCPP_ERROR_EXPRESSION(logger, gyroscope_id == -1, "Failed to find IMU sensor '%s' in MuJoCo model",
                          sensor_data.angular_velocity.name.c_str());
  RCLCPP_ERROR_EXPRESSION(logger, accelerometer_id == -1, "Failed to find IMU sensor '%s' in MuJoCo model",
                          sensor_data.linear_acceleration.name.c_str());

  if (quaternion_id == -1 || gyroscope_id == -1 || accelerometer_id == -1)
  {
    return std::nullopt;
  }

  sensor_data.orientation.mj_sensor_index = model->sensor_adr[quaternion_id];
  sensor_data.angular_velocity.mj_sensor_index = model->sensor_adr[gyroscope_id];
  sensor_data.linear_acceleration.mj_sensor_index = model->sensor_adr[accelerometer_id];
  return sensor_data;
}

}  // namespace mujoco_ros2_control::detail
