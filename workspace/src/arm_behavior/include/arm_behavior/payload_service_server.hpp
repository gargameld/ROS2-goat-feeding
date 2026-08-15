#ifndef ARM_BEHAVIOR__PAYLOAD_SERVICE_SERVER_HPP_
#define ARM_BEHAVIOR__PAYLOAD_SERVICE_SERVER_HPP_

#include <memory>

#include "arm_interface/srv/attach_object_to_gripper.hpp"
#include "arm_interface/srv/detach_object_from_gripper.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_behavior/payload_manager.hpp"

namespace arm
{

class PayloadServiceServer
{
public:
  PayloadServiceServer(rclcpp::Node::SharedPtr node, PayloadManager & payload_manager);

private:
  void attach(
    const std::shared_ptr<arm_interface::srv::AttachObjectToGripper::Request> request,
    std::shared_ptr<arm_interface::srv::AttachObjectToGripper::Response> response);
  void detach(
    const std::shared_ptr<arm_interface::srv::DetachObjectFromGripper::Request> request,
    std::shared_ptr<arm_interface::srv::DetachObjectFromGripper::Response> response);

  PayloadManager & payload_manager_;
  rclcpp::Service<arm_interface::srv::AttachObjectToGripper>::SharedPtr attach_service_;
  rclcpp::Service<arm_interface::srv::DetachObjectFromGripper>::SharedPtr detach_service_;
};

}  // namespace arm

#endif  // ARM_BEHAVIOR__PAYLOAD_SERVICE_SERVER_HPP_
