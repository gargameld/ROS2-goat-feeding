#include "arm_behavior/arm_action_server.hpp"

#include <cmath>
#include <utility>

namespace arm
{

ArmActionServer::ArmActionServer(
  rclcpp::Node::SharedPtr node,
  MotionExecutor & motion_executor)
: logger_(node->get_logger()), motion_executor_(motion_executor)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  move_to_pose_server_ = rclcpp_action::create_server<MoveToPose>(
    node, "move_arm_to_pose",
    std::bind(&ArmActionServer::handleMoveToPoseGoal, this, _1, _2),
    [this](const auto goal_handle) {return handleCancel(goal_handle);},
    std::bind(&ArmActionServer::handleMoveToPoseAccepted, this, _1));

  move_to_home_server_ = rclcpp_action::create_server<MoveToHome>(
    node, "move_arm_to_home_pose",
    std::bind(&ArmActionServer::handleMoveToHomeGoal, this, _1, _2),
    [this](const auto goal_handle) {return handleCancel(goal_handle);},
    std::bind(&ArmActionServer::handleMoveToHomeAccepted, this, _1));

  lift_server_ = rclcpp_action::create_server<LiftGripper>(
    node, "lift_gripper",
    std::bind(&ArmActionServer::handleLiftGoal, this, _1, _2),
    [this](const auto goal_handle) {return handleCancel(goal_handle);},
    std::bind(&ArmActionServer::handleLiftAccepted, this, _1));
}

ArmActionServer::~ArmActionServer()
{
  std::lock_guard<std::mutex> lock(workers_mutex_);
  for (auto & worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

rclcpp_action::GoalResponse ArmActionServer::handleMoveToPoseGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  if (!isValidPose(goal->target_pose)) {
    RCLCPP_WARN(logger_, "Rejected invalid move_arm_to_pose goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return reserveMotion() ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::GoalResponse ArmActionServer::handleMoveToHomeGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveToHome::Goal>)
{
  return reserveMotion() ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::GoalResponse ArmActionServer::handleLiftGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const LiftGripper::Goal> goal)
{
  if (!std::isfinite(goal->distance) || goal->distance <= 0.0) {
    RCLCPP_WARN(logger_, "Rejected lift distance; it must be positive and finite");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return reserveMotion() ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ArmActionServer::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandleBase>)
{
  // MoveGroupInterface execution is blocking; do not claim cancellation support.
  return rclcpp_action::CancelResponse::REJECT;
}

void ArmActionServer::handleMoveToPoseAccepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> goal_handle)
{
  launchWorker([this, goal_handle]() {
    executeGoal<MoveToPose>(goal_handle, [this, goal_handle]() {
      return motion_executor_.moveToPose(goal_handle->get_goal()->target_pose);
    });
  });
}

void ArmActionServer::handleMoveToHomeAccepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToHome>> goal_handle)
{
  launchWorker([this, goal_handle]() {
    executeGoal<MoveToHome>(goal_handle, [this]() {
      return motion_executor_.moveToHomePose();
    });
  });
}

void ArmActionServer::handleLiftAccepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<LiftGripper>> goal_handle)
{
  launchWorker([this, goal_handle]() {
    executeGoal<LiftGripper>(goal_handle, [this, goal_handle]() {
      return motion_executor_.lift(goal_handle->get_goal()->distance);
    });
  });
}

bool ArmActionServer::reserveMotion()
{
  bool expected = false;
  if (!motion_active_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(logger_, "Rejected arm goal because another motion is active");
    return false;
  }
  return true;
}

void ArmActionServer::launchWorker(std::function<void()> work)
{
  std::lock_guard<std::mutex> lock(workers_mutex_);
  workers_.emplace_back(std::move(work));
}

bool ArmActionServer::isValidPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & position = pose.position;
  const auto & orientation = pose.orientation;
  const bool finite =
    std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
    std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
    std::isfinite(orientation.z) && std::isfinite(orientation.w);
  const double quaternion_norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return finite && quaternion_norm_squared > 1e-12;
}

}  // namespace arm
