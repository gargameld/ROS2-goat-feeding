
#include "mujoco_ros2_control/system_interface/joint_actuator_mapping.hpp"

#include <algorithm>

namespace mujoco_ros2_control
{

void copy_actuator_states_to_joints(const std::vector<MuJoCoActuatorData>& actuators,
                                    std::vector<URDFJointData>& joints)
{
  for (auto& joint : joints)
  {
    std::for_each(actuators.begin(), actuators.end(), [&](const auto& actuator_interface) {
      if (actuator_interface.joint_name == joint.name)
      {
        joint.position_interface.state_ = actuator_interface.position_interface.state_;
        joint.velocity_interface.state_ = actuator_interface.velocity_interface.state_;
        joint.effort_interface.state_ = actuator_interface.effort_interface.state_;
      }
    });
  }
}

void copy_joint_commands_to_actuators(const std::vector<URDFJointData>& joints,
                                      std::vector<MuJoCoActuatorData>& actuators)
{
  for (const auto& joint : joints)
  {
    std::for_each(actuators.begin(), actuators.end(), [&](auto& actuator_interface) {
      if (actuator_interface.joint_name == joint.name && actuator_interface.actuator_type != ActuatorType::PASSIVE)
      {
        actuator_interface.position_interface.command_ = joint.position_interface.command_;
        actuator_interface.velocity_interface.command_ = joint.velocity_interface.command_;
        actuator_interface.effort_interface.command_ = joint.effort_interface.command_;
      }
    });
  }
}

}  // namespace mujoco_ros2_control
