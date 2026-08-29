#include "arm_behavior/payload_service_server.hpp"

#include <utility>

namespace arm
{

PayloadServiceServer::PayloadServiceServer(
  rclcpp::Node::SharedPtr node,
  PlanningSceneManager & planning_scene)
: planning_scene_(planning_scene)
{
  attach_service_ = node->create_service<arm_interface::srv::AttachObjectToGripper>(
    "attach_object_to_gripper",
    std::bind(&PayloadServiceServer::attach, this, std::placeholders::_1, std::placeholders::_2));
  detach_service_ = node->create_service<arm_interface::srv::DetachObjectFromGripper>(
    "detach_object_from_gripper",
    std::bind(&PayloadServiceServer::detach, this, std::placeholders::_1, std::placeholders::_2));
}

void PayloadServiceServer::attach(
  const std::shared_ptr<arm_interface::srv::AttachObjectToGripper::Request>,
  std::shared_ptr<arm_interface::srv::AttachObjectToGripper::Response> response)
{
  const auto result = planning_scene_.attachPayload();
  response->success = result.success;
  response->message = result.message;
}

void PayloadServiceServer::detach(
  const std::shared_ptr<arm_interface::srv::DetachObjectFromGripper::Request>,
  std::shared_ptr<arm_interface::srv::DetachObjectFromGripper::Response> response)
{
  const auto result = planning_scene_.detachPayload();
  response->success = result.success;
  response->message = result.message;
}

}  // namespace arm
