
#pragma once

#include <vector>

#include "mujoco_ros2_control/data.hpp"

namespace mujoco_ros2_control
{

/**
 * @brief Copy the actuator states onto the URDF joints that they drive.
 *
 * The read path: MuJoCo actuator states have already been pulled out of mjData, this hands them
 * to the joints exported as ros2_control state interfaces.
 */
void copy_actuator_states_to_joints(const std::vector<MuJoCoActuatorData>& actuators,
                                    std::vector<URDFJointData>& joints);

/**
 * @brief Copy the URDF joint commands onto the actuators that drive them.
 *
 * The write path: commands written by the controllers are handed to the actuators, from where
 * they are written into mjData. Passive actuators take no commands and are skipped.
 */
void copy_joint_commands_to_actuators(const std::vector<URDFJointData>& joints,
                                      std::vector<MuJoCoActuatorData>& actuators);

}  // namespace mujoco_ros2_control
