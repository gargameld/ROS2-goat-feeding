#include "arm_behavior/payload_manager.hpp"

#include <utility>

#include "shape_msgs/msg/solid_primitive.hpp"

namespace arm
{

PayloadManager::PayloadManager(
  std::shared_ptr<std::mutex> moveit_mutex,
  std::string tcp_link,
  std::string payload_id,
  std::array<double, 3> box_dimensions,
  std::vector<std::string> gripper_touch_links)
: moveit_mutex_(std::move(moveit_mutex)),
  planning_scene_("/"),
  tcp_link_(std::move(tcp_link)),
  payload_id_(std::move(payload_id)),
  box_dimensions_(box_dimensions),
  gripper_touch_links_(std::move(gripper_touch_links))
{
}

OperationResult PayloadManager::attach()
{
  std::lock_guard<std::mutex> lock(*moveit_mutex_);
  const auto attached_payload = makeAttachedPayload(moveit_msgs::msg::CollisionObject::ADD);
  if (!planning_scene_.applyAttachedCollisionObject(attached_payload)) {
    return {false, "Failed to attach the payload box to arm_tcp"};
  }
  return {true, "Payload box attached to arm_tcp"};
}

OperationResult PayloadManager::detach()
{
  std::lock_guard<std::mutex> lock(*moveit_mutex_);
  const auto attached_payload = makeAttachedPayload(moveit_msgs::msg::CollisionObject::REMOVE);
  if (!planning_scene_.applyAttachedCollisionObject(attached_payload)) {
    return {false, "Failed to detach the payload box from arm_tcp"};
  }
  return {true, "Payload box detached and left in the planning scene"};
}

moveit_msgs::msg::CollisionObject PayloadManager::makePayloadCollisionObject() const
{
  moveit_msgs::msg::CollisionObject payload;
  payload.header.frame_id = tcp_link_;
  payload.id = payload_id_;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {
    box_dimensions_[0],
    box_dimensions_[1],
    box_dimensions_[2]
  };

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.z = box_dimensions_[2] / 2.0;

  payload.primitives.push_back(box);
  payload.primitive_poses.push_back(box_pose);
  payload.operation = moveit_msgs::msg::CollisionObject::ADD;
  return payload;
}

moveit_msgs::msg::AttachedCollisionObject PayloadManager::makeAttachedPayload(
  std::int8_t operation) const
{
  moveit_msgs::msg::AttachedCollisionObject attached_payload;
  attached_payload.link_name = tcp_link_;
  attached_payload.touch_links = gripper_touch_links_;
  attached_payload.object = makePayloadCollisionObject();
  attached_payload.object.operation = operation;
  return attached_payload;
}

}  // namespace arm
