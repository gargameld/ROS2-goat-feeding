/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/system_interface/interface_export.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace mujoco_ros2_control
{

void append_joint_state_interfaces(std::vector<hardware_interface::StateInterface>& interfaces,
                                   std::vector<URDFJointData>& joints, const ComponentInfoMap& joint_hardware_info)
{
  for (auto& joint : joints)
  {
    if (auto it = joint_hardware_info.find(joint.name); it != joint_hardware_info.end())
    {
      for (const auto& state_if : it->second.state_interfaces)
      {
        if (state_if.name == hardware_interface::HW_IF_POSITION)
        {
          interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position_interface.state_);
        }
        else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
        {
          interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity_interface.state_);
        }
        else if (state_if.name == hardware_interface::HW_IF_EFFORT ||
                 state_if.name == hardware_interface::HW_IF_TORQUE || state_if.name == hardware_interface::HW_IF_FORCE)
        {
          interfaces.emplace_back(joint.name, state_if.name, &joint.effort_interface.state_);
        }
      }
    }
  }
}

void append_imu_state_interfaces(std::vector<hardware_interface::StateInterface>& interfaces,
                                 std::vector<IMUSensorData>& sensors, const ComponentInfoMap& sensor_hardware_info)
{
  for (auto& sensor : sensors)
  {
    if (auto it = sensor_hardware_info.find(sensor.name); it != sensor_hardware_info.end())
    {
      for (const auto& state_if : it->second.state_interfaces)
      {
        if (state_if.name == "orientation.x")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.orientation.data.x());
        }
        else if (state_if.name == "orientation.y")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.orientation.data.y());
        }
        else if (state_if.name == "orientation.z")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.orientation.data.z());
        }
        else if (state_if.name == "orientation.w")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.orientation.data.w());
        }
        else if (state_if.name == "angular_velocity.x")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.angular_velocity.data.x());
        }
        else if (state_if.name == "angular_velocity.y")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.angular_velocity.data.y());
        }
        else if (state_if.name == "angular_velocity.z")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.angular_velocity.data.z());
        }
        else if (state_if.name == "linear_acceleration.x")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.linear_acceleration.data.x());
        }
        else if (state_if.name == "linear_acceleration.y")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.linear_acceleration.data.y());
        }
        else if (state_if.name == "linear_acceleration.z")
        {
          interfaces.emplace_back(sensor.name, state_if.name, &sensor.linear_acceleration.data.z());
        }
      }
    }
  }
}

void append_joint_command_interfaces(std::vector<hardware_interface::CommandInterface>& interfaces,
                                     std::vector<URDFJointData>& joints, const ComponentInfoMap& joint_hardware_info)
{
  for (auto& joint : joints)
  {
    if (auto it = joint_hardware_info.find(joint.name); it != joint_hardware_info.end())
    {
      for (const auto& command_if : it->second.command_interfaces)
      {
        if (command_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
        {
          interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position_interface.command_);
        }
        else if (command_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
        {
          interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity_interface.command_);
        }
        else if (command_if.name == hardware_interface::HW_IF_EFFORT ||
                 command_if.name == hardware_interface::HW_IF_TORQUE ||
                 command_if.name == hardware_interface::HW_IF_FORCE)
        {
          interfaces.emplace_back(joint.name, command_if.name, &joint.effort_interface.command_);
        }
      }
    }
  }
}

}  // namespace mujoco_ros2_control
