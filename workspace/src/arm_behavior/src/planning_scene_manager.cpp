#include "arm_behavior/planning_scene_manager.hpp"

#include <chrono>
#include <sstream>
#include <utility>

#include "moveit/collision_detection/collision_matrix.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace
{

// Wall-clock allowance for the round trip, which also covers move_group
// locking the planning scene while a plan is in flight.
constexpr std::chrono::seconds kPlanningSceneServiceTimeout{5};
constexpr std::chrono::seconds kClearOctomapServiceTimeout{5};

rclcpp::Logger logger()
{
  return rclcpp::get_logger("arm.planning_scene_manager");
}

std::string join(const std::vector<std::string> & names)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index > 0U) {
      stream << ", ";
    }
    stream << names[index];
  }
  return stream.str();
}

std::string join(const std::vector<std::pair<std::string, std::string>> & pairs)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < pairs.size(); ++index) {
    if (index > 0U) {
      stream << ", ";
    }
    stream << pairs[index].first << '/' << pairs[index].second;
  }
  return stream.str();
}

}  // namespace

namespace arm
{

bool CollisionExceptions::empty() const
{
  return world_object_ids.empty() && payload_object_ids.empty() &&
         self_collision_link_pairs.empty();
}

PlanningSceneManager::CollisionScope::CollisionScope(
  PlanningSceneManager & manager,
  std::optional<moveit_msgs::msg::AllowedCollisionMatrix> previous_acm,
  std::string motion_description)
: manager_(manager),
  previous_acm_(std::move(previous_acm)),
  motion_description_(std::move(motion_description))
{
}

PlanningSceneManager::CollisionScope::~CollisionScope()
{
  if (previous_acm_) {
    manager_.restoreAllowedCollisions(*previous_acm_, motion_description_);
  }
}

bool PlanningSceneManager::CollisionScope::active() const
{
  return previous_acm_.has_value();
}

PlanningSceneManager::PlanningSceneManager(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<std::mutex> moveit_mutex,
  std::string tcp_link,
  PayloadDescription payload)
: node_(std::move(node)),
  planning_scene_("/"),
  moveit_mutex_(std::move(moveit_mutex)),
  tcp_link_(std::move(tcp_link)),
  payload_(std::move(payload))
{
  get_planning_scene_client_ =
    node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  apply_planning_scene_client_ =
    node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
  clear_octomap_client_ = node_->create_client<std_srvs::srv::Empty>("clear_octomap");
}

OperationResult PlanningSceneManager::attachPayload()
{
  std::lock_guard<std::mutex> lock(*moveit_mutex_);
  const auto attached_payload = makeAttachedPayload(moveit_msgs::msg::CollisionObject::ADD);
  if (!planning_scene_.applyAttachedCollisionObject(attached_payload)) {
    return {false, "Failed to attach the payload box to " + tcp_link_};
  }
  RCLCPP_INFO(
    logger(), "Attached the payload box '%s' to %s", payload_.id.c_str(), tcp_link_.c_str());
  return {true, "Payload box attached to " + tcp_link_};
}

OperationResult PlanningSceneManager::detachPayload()
{
  std::lock_guard<std::mutex> lock(*moveit_mutex_);
  const auto attached_payload = makeAttachedPayload(moveit_msgs::msg::CollisionObject::REMOVE);
  if (!planning_scene_.applyAttachedCollisionObject(attached_payload)) {
    return {false, "Failed to detach the payload box from " + tcp_link_};
  }
  RCLCPP_INFO(
    logger(), "Detached the payload box '%s' from %s", payload_.id.c_str(), tcp_link_.c_str());
  return {true, "Payload box detached and left in the planning scene"};
}

moveit_msgs::msg::CollisionObject PlanningSceneManager::makePayloadCollisionObject() const
{
  moveit_msgs::msg::CollisionObject payload;
  payload.header.frame_id = tcp_link_;
  payload.id = payload_.id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {
    payload_.box_dimensions[0],
    payload_.box_dimensions[1],
    payload_.box_dimensions[2]
  };

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.z = payload_.box_dimensions[2] / 2.0;

  payload.primitives.push_back(box);
  payload.primitive_poses.push_back(box_pose);
  payload.operation = moveit_msgs::msg::CollisionObject::ADD;
  return payload;
}

moveit_msgs::msg::AttachedCollisionObject PlanningSceneManager::makeAttachedPayload(
  std::int8_t operation) const
{
  moveit_msgs::msg::AttachedCollisionObject attached_payload;
  attached_payload.link_name = tcp_link_;
  attached_payload.touch_links = payload_.gripper_touch_links;
  attached_payload.object = makePayloadCollisionObject();
  attached_payload.object.operation = operation;
  return attached_payload;
}

std::optional<moveit_msgs::msg::AllowedCollisionMatrix>
PlanningSceneManager::currentAllowedCollisionMatrix()
{
  // Every failure below leaves the scene untouched, which only means the
  // motion is planned with the usual collisions still checked: it can fail to
  // plan, but it can never be planned through geometry that was meant to be
  // honored.
  if (!get_planning_scene_client_->wait_for_service(kPlanningSceneServiceTimeout)) {
    RCLCPP_WARN(
      logger(),
      "'get_planning_scene' is unavailable; planning with every collision still checked");
    return std::nullopt;
  }

  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = get_planning_scene_client_->async_send_request(request);
  if (future.wait_for(kPlanningSceneServiceTimeout) != std::future_status::ready) {
    get_planning_scene_client_->remove_pending_request(future);
    RCLCPP_WARN(
      logger(),
      "'get_planning_scene' did not answer within %ld s; planning with every collision still "
      "checked", static_cast<long>(kPlanningSceneServiceTimeout.count()));
    return std::nullopt;
  }

  const auto previous_acm = future.get()->scene.allowed_collision_matrix;
  // An empty matrix cannot be restored: applying it back as a diff would be a
  // no-op and the relaxed pairs would stay ignored for every later motion.
  if (previous_acm.entry_names.empty()) {
    RCLCPP_WARN(
      logger(),
      "The planning scene reported an empty allowed collision matrix; planning with every "
      "collision still checked");
    return std::nullopt;
  }
  return previous_acm;
}

PlanningSceneManager::CollisionScope PlanningSceneManager::allowCollisions(
  const CollisionExceptions & exceptions,
  const std::string & motion_description)
{
  if (exceptions.empty()) {
    return CollisionScope(*this, std::nullopt, motion_description);
  }

  auto previous_acm = currentAllowedCollisionMatrix();
  if (!previous_acm) {
    return CollisionScope(*this, std::nullopt, motion_description);
  }

  collision_detection::AllowedCollisionMatrix acm(*previous_acm);
  for (const auto & world_object_id : exceptions.world_object_ids) {
    // setEntry covers the pairs the matrix already knows about (the robot
    // links); setDefaultEntry covers everything else the scene may hold, such
    // as the attached payload and the octomap.
    acm.setEntry(world_object_id, true);
    acm.setDefaultEntry(world_object_id, true);
  }
  for (const auto & payload_object_id : exceptions.payload_object_ids) {
    // One pair per object, so every robot link keeps being checked against
    // these boxes; only the attached payload stops being.
    acm.setEntry(payload_.id, payload_object_id, true);
  }
  for (const auto & link_pair : exceptions.self_collision_link_pairs) {
    // One robot link against one other robot link, exactly as an SRDF
    // disable_collisions entry would, but only for this motion.
    acm.setEntry(link_pair.first, link_pair.second, true);
  }

  moveit_msgs::msg::AllowedCollisionMatrix updated_acm;
  acm.getMessage(updated_acm);
  const std::string description =
    "collisions with [" + join(exceptions.world_object_ids) + "], between '" + payload_.id +
    "' and [" + join(exceptions.payload_object_ids) + "], and between the link pairs [" +
    join(exceptions.self_collision_link_pairs) + "]";
  if (!applyAllowedCollisionMatrix(updated_acm)) {
    RCLCPP_WARN(
      logger(), "Failed to ignore %s; planning with them still checked", description.c_str());
    return CollisionScope(*this, std::nullopt, motion_description);
  }
  RCLCPP_INFO(
    logger(), "Ignoring %s for the %s motion", description.c_str(), motion_description.c_str());
  return CollisionScope(*this, std::move(previous_acm), motion_description);
}

void PlanningSceneManager::restoreAllowedCollisions(
  const moveit_msgs::msg::AllowedCollisionMatrix & allowed_collision_matrix,
  const std::string & motion_description)
{
  if (!applyAllowedCollisionMatrix(allowed_collision_matrix)) {
    RCLCPP_ERROR(
      logger(),
      "Failed to restore the allowed collision matrix; the collisions relaxed for the %s motion "
      "stay ignored until the planning scene is reloaded", motion_description.c_str());
    return;
  }
  RCLCPP_INFO(
    logger(), "Restored the allowed collision matrix after the %s motion",
    motion_description.c_str());
}

bool PlanningSceneManager::applyAllowedCollisionMatrix(
  const moveit_msgs::msg::AllowedCollisionMatrix & allowed_collision_matrix)
{
  if (!apply_planning_scene_client_->wait_for_service(kPlanningSceneServiceTimeout)) {
    RCLCPP_WARN(logger(), "'apply_planning_scene' is unavailable");
    return false;
  }

  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene.is_diff = true;
  request->scene.robot_state.is_diff = true;
  request->scene.allowed_collision_matrix = allowed_collision_matrix;
  auto future = apply_planning_scene_client_->async_send_request(request);
  if (future.wait_for(kPlanningSceneServiceTimeout) != std::future_status::ready) {
    apply_planning_scene_client_->remove_pending_request(future);
    RCLCPP_WARN(
      logger(), "'apply_planning_scene' did not answer within %ld s",
      static_cast<long>(kPlanningSceneServiceTimeout.count()));
    return false;
  }
  if (!future.get()->success) {
    RCLCPP_WARN(logger(), "MoveIt rejected the allowed collision matrix diff");
    return false;
  }
  return true;
}

bool PlanningSceneManager::clearOctomap()
{
  if (!clear_octomap_client_->wait_for_service(kClearOctomapServiceTimeout)) {
    RCLCPP_ERROR(
      logger(), "'clear_octomap' is unavailable after %ld s",
      static_cast<long>(kClearOctomapServiceTimeout.count()));
    return false;
  }

  auto future = clear_octomap_client_->async_send_request(
    std::make_shared<std_srvs::srv::Empty::Request>());
  if (future.wait_for(kClearOctomapServiceTimeout) != std::future_status::ready) {
    clear_octomap_client_->remove_pending_request(future);
    RCLCPP_ERROR(
      logger(), "'clear_octomap' did not answer within %ld s",
      static_cast<long>(kClearOctomapServiceTimeout.count()));
    return false;
  }
  future.get();
  RCLCPP_INFO(logger(), "Cleared the octomap");
  return true;
}

}  // namespace arm
