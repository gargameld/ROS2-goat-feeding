#ifndef ARM_BEHAVIOR__PAYLOAD_MANAGER_HPP_
#define ARM_BEHAVIOR__PAYLOAD_MANAGER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"

#include "arm_behavior/operation_result.hpp"

namespace arm
{

class PayloadManager
{
public:
  PayloadManager(
    std::shared_ptr<std::mutex> moveit_mutex,
    std::string tcp_link,
    std::string payload_id,
    std::array<double, 3> box_dimensions,
    std::vector<std::string> gripper_touch_links);

  OperationResult attach();
  OperationResult detach();

private:
  moveit_msgs::msg::CollisionObject makePayloadCollisionObject() const;
  moveit_msgs::msg::AttachedCollisionObject makeAttachedPayload(std::int8_t operation) const;

  std::shared_ptr<std::mutex> moveit_mutex_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  std::string tcp_link_;
  std::string payload_id_;
  std::array<double, 3> box_dimensions_;
  std::vector<std::string> gripper_touch_links_;
};

}  // namespace arm

#endif  // ARM_BEHAVIOR__PAYLOAD_MANAGER_HPP_
