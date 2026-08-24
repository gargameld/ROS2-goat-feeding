#ifndef ARM_BEHAVIOR__SIMULATION_CONTROL_HPP_
#define ARM_BEHAVIOR__SIMULATION_CONTROL_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace arm
{

void pause_simulation(
  const rclcpp::Node::SharedPtr & node,
  std::chrono::seconds timeout = std::chrono::seconds(10));

void resume_simulation(
  const rclcpp::Node::SharedPtr & node,
  std::chrono::seconds timeout = std::chrono::seconds(10));

}  // namespace arm

#endif  // ARM_BEHAVIOR__SIMULATION_CONTROL_HPP_
