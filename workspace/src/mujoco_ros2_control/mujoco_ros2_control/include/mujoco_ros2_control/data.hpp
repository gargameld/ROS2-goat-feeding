
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <string>

namespace mujoco_ros2_control
{

/**
 * Maps to MuJoCo actuator types:
 *  - MOTOR for MuJoCo motor actuator
 *  - POSITION for MuJoCo position actuator
 *  - VELOCITY for MuJoCo velocity actuator
 *  - CUSTOM  for MuJoCo general actuator or other types
 *
 * \note the MuJoCo types are as per the MuJoCo documentation:
 * https://mujoco.readthedocs.io/en/latest/XMLreference.html#actuator
 */

enum class ActuatorType
{
  UNKNOWN,
  MOTOR,
  POSITION,
  VELOCITY,
  PASSIVE,
  CUSTOM
};

/**
 * Data structure for each command/state interface.
 */
struct InterfaceData
{
  double command_ = std::numeric_limits<double>::quiet_NaN();
  double state_ = std::numeric_limits<double>::quiet_NaN();
};

/**
 * Wrapper for MuJoCo actuators and relevant ROS HW interface data.
 * @param joint_name Name of the MuJoCo joint handled by the actuator.
 * @param position_interface Data for position command/state interface.
 * @param velocity_interface Data for velocity command/state interface.
 * @param effort_interface Data for effort command/state interface.
 * @param actuator_type Type of the MuJoCo actuator.
 * @param mj_pos_adr MuJoCo position address in mjData->qpos.
 * @param mj_vel_adr MuJoCo velocity address in mjData->qvel.
 * @param mj_actuator_id MuJoCo actuator id as per mjModel->actuator_id.
 * @param is_position_control_enabled Boolean flag indicating if position control is enabled.
 * @param is_velocity_control_enabled Boolean flag indicating if velocity control is enabled.
 * @param is_effort_control_enabled Boolean flag indicating if effort control is enabled.
 */
struct MuJoCoActuatorData
{
  std::string joint_name = "";
  InterfaceData position_interface;
  InterfaceData velocity_interface;
  InterfaceData effort_interface;
  ActuatorType actuator_type{ ActuatorType::UNKNOWN };
  int mj_pos_adr = -1;
  int mj_vel_adr = -1;
  int mj_actuator_id = -1;

  // Booleans record whether or not we should be writing commands to these interfaces
  // based on if they have been claimed.
  bool is_position_control_enabled{ false };
  bool is_velocity_control_enabled{ false };
  bool is_effort_control_enabled{ false };

  void copy_command_to_state()
  {
    position_interface.state_ = position_interface.command_;
    velocity_interface.state_ = velocity_interface.command_;
    effort_interface.state_ = effort_interface.command_;
  }
};

/**
 * Structure for the URDF joint data.
 * @param name Name of the joint.
 * @param position_interface Data for position command/state interface.
 * @param velocity_interface Data for velocity command/state interface.
 * @param effort_interface Data for effort command/state interface.
 * @param is_position_control_enabled Boolean flag indicating if position control is enabled.
 * @param is_velocity_control_enabled Boolean flag indicating if velocity control is enabled.
 * @param is_effort_control_enabled Boolean flag indicating if effort control is enabled.
 */
struct URDFJointData
{
  std::string name = "";
  InterfaceData position_interface;
  InterfaceData velocity_interface;
  InterfaceData effort_interface;

  bool is_position_control_enabled{ false };
  bool is_velocity_control_enabled{ false };
  bool is_effort_control_enabled{ false };
};

template <typename T>
struct SensorData
{
  std::string name;
  T data;
  int mj_sensor_index;
};

struct IMUSensorData
{
  std::string name;
  SensorData<Eigen::Quaternion<double>> orientation;
  SensorData<Eigen::Vector3d> angular_velocity;
  SensorData<Eigen::Vector3d> linear_acceleration;
};

}  // namespace mujoco_ros2_control
