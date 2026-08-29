#ifndef ARM_BEHAVIOR__MOTION_EXECUTOR_HPP_
#define ARM_BEHAVIOR__MOTION_EXECUTOR_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"

#include "arm_behavior/operation_result.hpp"
#include "arm_behavior/planning_scene_manager.hpp"

namespace arm
{

class MotionExecutor
{
public:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  MotionExecutor(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<MoveGroupInterface> move_group,
    std::shared_ptr<std::mutex> moveit_mutex,
    PlanningSceneManager & planning_scene,
    std::string tcp_link,
    std::string home_pose_name);

  OperationResult moveToPose(
    const geometry_msgs::msg::Pose & target_pose,
    const std::string & reference_frame);
  OperationResult checkPoseReachability(
    const geometry_msgs::msg::Pose & target_pose,
    const std::string & reference_frame);
  OperationResult moveToHomePose();
  OperationResult lift(double distance);

private:
  void log_path(
    const MoveGroupInterface::Plan & plan,
    const std::string & motion_description) const;
  void log_planning_context(const std::string & motion_description) const;
  OperationResult planAndExecute(const std::string & motion_description);
  OperationResult planOnly(const std::string & motion_description);
  OperationResult checkGoalStateValidity(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & motion_description);
  OperationResult setPoseTarget(
    const geometry_msgs::msg::Pose & target_pose,
    const std::string & reference_frame,
    std::string & target_frame);

  std::shared_ptr<MoveGroupInterface> move_group_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<std::mutex> moveit_mutex_;
  PlanningSceneManager & planning_scene_;
  std::string tcp_link_;
  std::string home_pose_name_;
};

}  // namespace arm

#endif  // ARM_BEHAVIOR__MOTION_EXECUTOR_HPP_
