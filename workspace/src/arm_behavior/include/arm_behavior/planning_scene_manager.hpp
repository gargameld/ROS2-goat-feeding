#ifndef ARM_BEHAVIOR__PLANNING_SCENE_MANAGER_HPP_
#define ARM_BEHAVIOR__PLANNING_SCENE_MANAGER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/allowed_collision_matrix.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "arm_behavior/operation_result.hpp"

namespace arm
{

// The box that stands in for whatever the gripper is carrying. It is a coarse
// envelope around the jaws rather than the food cube itself, so that a grasped
// object the sensors can no longer see is still collision-checked against the
// rest of the scene.
struct PayloadDescription
{
  std::string id;
  std::array<double, 3> box_dimensions;
  // Robot links the payload is allowed to rest against once attached: the
  // gripper links that are holding it.
  std::vector<std::string> gripper_touch_links;
};

// The collision checks one motion needs relaxed. Everything left out keeps
// being checked as the SRDF and the planning scene describe it.
struct CollisionExceptions
{
  // World objects that stop being checked against everything in the planning
  // scene, robot links included.
  std::vector<std::string> world_object_ids;
  // World objects that stop being checked against the attached payload alone,
  // so every robot link is still checked against them.
  std::vector<std::string> payload_object_ids;
  // Robot link pairs whose self-collision check is dropped on top of the pairs
  // the SRDF already disables. Both names must be links the planning scene
  // knows, and the order of the two does not matter.
  std::vector<std::pair<std::string, std::string>> self_collision_link_pairs;

  bool empty() const;
};

// Owns every change this node makes to the move_group planning scene: the
// attached payload, the allowed collision matrix (world, payload and self
// collisions) and the octomap. Callers describe what they need; how that
// reaches move_group, and how the scene is put back afterwards, lives here.
class PlanningSceneManager
{
public:
  // Holds the allowed collision matrix as it was before a relaxation and puts
  // it back when it goes out of scope, including when the motion in between
  // throws. A scope that could not change the scene restores nothing, which
  // leaves the motion planned with every collision checked as usual: it can
  // fail to plan, but it can never be planned through geometry that was meant
  // to be honored.
  class CollisionScope
  {
  public:
    ~CollisionScope();
    CollisionScope(const CollisionScope &) = delete;
    CollisionScope & operator=(const CollisionScope &) = delete;
    CollisionScope(CollisionScope &&) = delete;
    CollisionScope & operator=(CollisionScope &&) = delete;

    // Whether the planning scene was actually relaxed. A false scope is not an
    // error: the caller may still plan, only with the checks left in place.
    bool active() const;

  private:
    friend class PlanningSceneManager;

    CollisionScope(
      PlanningSceneManager & manager,
      std::optional<moveit_msgs::msg::AllowedCollisionMatrix> previous_acm,
      std::string motion_description);

    PlanningSceneManager & manager_;
    std::optional<moveit_msgs::msg::AllowedCollisionMatrix> previous_acm_;
    std::string motion_description_;
  };

  PlanningSceneManager(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<std::mutex> moveit_mutex,
    std::string tcp_link,
    PayloadDescription payload);

  // Attaches the payload box to the TCP link, so it travels with the gripper
  // and is collision-checked against the world for every motion planned from
  // here on.
  OperationResult attachPayload();
  // Detaches the payload box, which stays in the planning scene as a world
  // object at the pose it was released in.
  OperationResult detachPayload();

  // Relaxes collision checking for the lifetime of the returned scope, which
  // must therefore outlive the motion it is meant to cover.
  [[nodiscard]] CollisionScope allowCollisions(
    const CollisionExceptions & exceptions,
    const std::string & motion_description);

  // Drops every occupied voxel from the move_group octomap. Returns false if
  // the service could not be reached, i.e. if the octomap may still hold
  // stale geometry.
  bool clearOctomap();

private:
  moveit_msgs::msg::CollisionObject makePayloadCollisionObject() const;
  moveit_msgs::msg::AttachedCollisionObject makeAttachedPayload(std::int8_t operation) const;
  // Reads the planning scene's allowed collision matrix, or nothing if it
  // could not be read back in a form that can later be restored.
  std::optional<moveit_msgs::msg::AllowedCollisionMatrix> currentAllowedCollisionMatrix();
  void restoreAllowedCollisions(
    const moveit_msgs::msg::AllowedCollisionMatrix & allowed_collision_matrix,
    const std::string & motion_description);
  bool applyAllowedCollisionMatrix(
    const moveit_msgs::msg::AllowedCollisionMatrix & allowed_collision_matrix);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_client_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  std::shared_ptr<std::mutex> moveit_mutex_;
  std::string tcp_link_;
  PayloadDescription payload_;
};

}  // namespace arm

#endif  // ARM_BEHAVIOR__PLANNING_SCENE_MANAGER_HPP_
