
#include "mujoco_ros2_control/system_interface/joint_command_setup.hpp"

#include "mujoco_ros2_control/system_interface/mujoco_actuator_discovery.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>

#include <hardware_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{
namespace
{

std::vector<std::string> get_interfaces_in_order(const std::vector<std::string>& available_interfaces,
                                                 const std::vector<std::string>& desired_order)
{
  std::vector<std::string> ordered_interfaces;
  for (const auto& interface : desired_order)
  {
    if (std::find(available_interfaces.begin(), available_interfaces.end(), interface) != available_interfaces.end())
    {
      ordered_interfaces.push_back(interface);
    }
  }
  // Anything the joint declares that is not in the desired order still gets exported, after it.
  for (const auto& interface : available_interfaces)
  {
    ros2_control::add_item(ordered_interfaces, interface);
  }
  return ordered_interfaces;
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
    RCLCPP_ERROR(logger,
                 "Position command interface for the joint : %s requires a MuJoCo position actuator",
                 actuator_name.c_str());
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
    RCLCPP_ERROR(logger,
                 "Velocity command interface for the joint : %s requires a MuJoCo velocity actuator",
                 actuator_name.c_str());
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

}  // namespace

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
      !actuator.is_velocity_control_enabled && !actuator.is_effort_control_enabled)
  {
    throw std::runtime_error("Joint '" + joint.name + "' which uses actuator '" + actuator_name +
                             "' has an unsupported command interface for the specified MuJoCo actuator");
  }
}

void register_urdf_joints(const hardware_interface::HardwareInfo& hardware_info, const mjModel* model,
                          std::vector<MuJoCoActuatorData>& actuators, std::vector<URDFJointData>& joints,
                          ComponentInfoMap& joint_hardware_info, const rclcpp::Logger& logger)
{
  RCLCPP_INFO(logger, "Registering joints...");
  joints.resize(hardware_info.joints.size());

  for (size_t joint_index = 0; joint_index < hardware_info.joints.size(); joint_index++)
  {
    auto joint = hardware_info.joints.at(joint_index);

    // Get the information for the URDF Joint data
    URDFJointData& joint_data = joints.at(joint_index);
    joint_data.name = joint.name;

    auto* actuator = find_controllable_actuator(actuators, model, joint.name);
    const bool actuator_exists = actuator != nullptr;
    // This isn't a failure the joint just won't be controllable
    RCLCPP_INFO_EXPRESSION(logger, !actuator_exists,
                           "Failed to find actuator for joint : %s. This joint will be treated as a passive joint.",
                           joint.name.c_str());
    RCLCPP_INFO_EXPRESSION(logger, joint.command_interfaces.empty(), "Joint : %s is a passive joint",
                           joint.name.c_str());
    if (!joint.command_interfaces.empty() && !actuator_exists)
    {
      RCLCPP_ERROR(logger,
                   "Joint : %s has command interfaces defined but no matching actuator in the MuJoCo model. This joint "
                   "will be treated as a passive joint and no command interfaces will be exported.",
                   joint.name.c_str());
      joint.command_interfaces.clear();
    }

    // Add to the joint hw information map
    joint_hardware_info.insert(std::make_pair(joint.name, joint));

    // Set initial values to joint interfaces if they are set in the info
    initialize_joint_interfaces(joint, joint_data);

    const auto command_interface_names = get_ordered_command_interfaces(joint);

    if (actuator)
    {
      configure_joint_command_interfaces(joint, joint.name, command_interface_names, *actuator, logger);
    }
  }
}

}  // namespace mujoco_ros2_control
