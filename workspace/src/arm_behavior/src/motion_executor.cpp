#include "arm_behavior/motion_executor.hpp"

#include <utility>

#include "moveit/move_group_interface/move_group_interface.hpp"

namespace arm
{

MotionExecutor::MotionExecutor(
  std::shared_ptr<MoveGroupInterface> move_group,
  std::shared_ptr<std::mutex> moveit_mutex,
  std::string tcp_link,
  std::string home_pose_name)
: move_group_(std::move(move_group)),
  moveit_mutex_(std::move(moveit_mutex)),
  tcp_link_(std::move(tcp_link)),
  home_pose_name_(std::move(home_pose_name))
{
}

OperationResult MotionExecutor::moveToPose(const geometry_msgs::msg::Pose & target_pose)
{
  std::lock_guard<std::mutex> lock(*moveit_mutex_);
  move_group_->setStartStateToCurrentState();
  move_group_->clearPoseTargets();
  if (!move_group_->setPoseTarget(target_pose, tcp_link_)) {
    return {false, "MoveIt rejected the arm_tcp target pose"};
  }
  return planAndExecute("target pose");
}

OperationResult MotionExecutor::moveToHomePose()
{
  std::lock_guard<std::mutex> lock(*moveit_mutex_);
  move_group_->setStartStateToCurrentState();
  move_group_->clearPoseTargets();
  if (!move_group_->setNamedTarget(home_pose_name_)) {
    return {false, "Named SRDF state '" + home_pose_name_ + "' was not found"};
  }
  return planAndExecute("home pose");
}

OperationResult MotionExecutor::lift(double distance)
{
  std::lock_guard<std::mutex> lock(*moveit_mutex_);
  auto target_pose = move_group_->getCurrentPose(tcp_link_).pose;
  target_pose.position.z += distance;

  move_group_->setStartStateToCurrentState();
  move_group_->clearPoseTargets();
  if (!move_group_->setPoseTarget(target_pose, tcp_link_)) {
    return {false, "MoveIt rejected the lifted arm_tcp target pose"};
  }
  return planAndExecute("lift");
}

OperationResult MotionExecutor::planAndExecute(const std::string & motion_description)
{
  MoveGroupInterface::Plan plan;
  const auto planning_result = move_group_->plan(plan);
  if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_->clearPoseTargets();
    return {false, "Failed to plan arm motion to " + motion_description};
  }

  const auto execution_result = move_group_->execute(plan);
  move_group_->clearPoseTargets();
  if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
    return {false, "Failed to execute arm motion to " + motion_description};
  }
  return {true, "Arm motion to " + motion_description + " completed"};
}

}  // namespace arm
