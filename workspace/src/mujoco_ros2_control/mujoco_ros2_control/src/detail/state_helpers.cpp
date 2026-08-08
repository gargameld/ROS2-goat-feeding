/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "mujoco_ros2_control/detail/state_helpers.hpp"

namespace mujoco_ros2_control::detail
{

void read_actuator_states(const mjData* control_data, std::vector<MuJoCoActuatorData>& actuators,
                          sensor_msgs::msg::JointState& actuator_state_message)
{
  actuator_state_message.position.clear();
  actuator_state_message.velocity.clear();
  actuator_state_message.effort.clear();
  for (auto& actuator_state : actuators)
  {
    actuator_state.position_interface.state_ = control_data->qpos[actuator_state.mj_pos_adr];
    actuator_state.velocity_interface.state_ = control_data->qvel[actuator_state.mj_vel_adr];
    actuator_state.effort_interface.state_ = control_data->qfrc_actuator[actuator_state.mj_vel_adr];
    actuator_state_message.position.push_back(actuator_state.position_interface.state_);
    actuator_state_message.velocity.push_back(actuator_state.velocity_interface.state_);
    actuator_state_message.effort.push_back(actuator_state.effort_interface.state_);
  }
}

void read_imu_states(const mjData* control_data, std::vector<IMUSensorData>& sensors)
{
  for (auto& data : sensors)
  {
    data.orientation.data.w() = control_data->sensordata[data.orientation.mj_sensor_index];
    data.orientation.data.x() = control_data->sensordata[data.orientation.mj_sensor_index + 1];
    data.orientation.data.y() = control_data->sensordata[data.orientation.mj_sensor_index + 2];
    data.orientation.data.z() = control_data->sensordata[data.orientation.mj_sensor_index + 3];

    data.angular_velocity.data.x() = control_data->sensordata[data.angular_velocity.mj_sensor_index];
    data.angular_velocity.data.y() = control_data->sensordata[data.angular_velocity.mj_sensor_index + 1];
    data.angular_velocity.data.z() = control_data->sensordata[data.angular_velocity.mj_sensor_index + 2];

    data.linear_acceleration.data.x() = control_data->sensordata[data.linear_acceleration.mj_sensor_index];
    data.linear_acceleration.data.y() = control_data->sensordata[data.linear_acceleration.mj_sensor_index + 1];
    data.linear_acceleration.data.z() = control_data->sensordata[data.linear_acceleration.mj_sensor_index + 2];
  }
}

void read_force_torque_states(const mjData* control_data, std::vector<FTSensorData>& sensors)
{
  for (auto& data : sensors)
  {
    data.force.data.x() = -control_data->sensordata[data.force.mj_sensor_index];
    data.force.data.y() = -control_data->sensordata[data.force.mj_sensor_index + 1];
    data.force.data.z() = -control_data->sensordata[data.force.mj_sensor_index + 2];

    data.torque.data.x() = -control_data->sensordata[data.torque.mj_sensor_index];
    data.torque.data.y() = -control_data->sensordata[data.torque.mj_sensor_index + 1];
    data.torque.data.z() = -control_data->sensordata[data.torque.mj_sensor_index + 2];
  }
}

void populate_floating_base_odometry(const mjData* control_data, int qpos_address, int qvel_address,
                                     nav_msgs::msg::Odometry& message)
{
  message.pose.pose.position.x = control_data->qpos[qpos_address];
  message.pose.pose.position.y = control_data->qpos[qpos_address + 1];
  message.pose.pose.position.z = control_data->qpos[qpos_address + 2];

  message.pose.pose.orientation.w = control_data->qpos[qpos_address + 3];
  message.pose.pose.orientation.x = control_data->qpos[qpos_address + 4];
  message.pose.pose.orientation.y = control_data->qpos[qpos_address + 5];
  message.pose.pose.orientation.z = control_data->qpos[qpos_address + 6];

  message.twist.twist.linear.x = control_data->qvel[qvel_address];
  message.twist.twist.linear.y = control_data->qvel[qvel_address + 1];
  message.twist.twist.linear.z = control_data->qvel[qvel_address + 2];

  message.twist.twist.angular.x = control_data->qvel[qvel_address + 3];
  message.twist.twist.angular.y = control_data->qvel[qvel_address + 4];
  message.twist.twist.angular.z = control_data->qvel[qvel_address + 5];
}

void update_mimic_joint_commands(std::vector<URDFJointData>& joints)
{
  for (auto& joint : joints)
  {
    if (joint.is_mimic)
    {
      joint.position_interface.command_ =
          joint.mimic_multiplier * joints.at(joint.mimicked_joint_index).position_interface.command_;
      joint.velocity_interface.command_ =
          joint.mimic_multiplier * joints.at(joint.mimicked_joint_index).velocity_interface.command_;
      joint.effort_interface.command_ =
          joint.mimic_multiplier * joints.at(joint.mimicked_joint_index).effort_interface.command_;
    }
  }
}

}  // namespace mujoco_ros2_control::detail
