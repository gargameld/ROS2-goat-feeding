#ifndef ARM_BEHAVIOR__ARM_ACTION_SERVER_HPP_
#define ARM_BEHAVIOR__ARM_ACTION_SERVER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "arm_interface/action/check_pose_reachability.hpp"
#include "arm_interface/action/lift_gripper.hpp"
#include "arm_interface/action/move_arm_to_home_pose.hpp"
#include "arm_interface/action/move_arm_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm_behavior/motion_executor.hpp"

namespace arm
{

class ArmActionServer
{
public:
  ArmActionServer(rclcpp::Node::SharedPtr node, MotionExecutor & motion_executor);
  ~ArmActionServer();

private:
  using MoveToPose = arm_interface::action::MoveArmToPose;
  using CheckPoseReachability = arm_interface::action::CheckPoseReachability;
  using MoveToHome = arm_interface::action::MoveArmToHomePose;
  using LiftGripper = arm_interface::action::LiftGripper;

  rclcpp_action::GoalResponse handleMoveToPoseGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal);
  rclcpp_action::GoalResponse handleCheckPoseReachabilityGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CheckPoseReachability::Goal> goal);
  rclcpp_action::GoalResponse handleMoveToHomeGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToHome::Goal> goal);
  rclcpp_action::GoalResponse handleLiftGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const LiftGripper::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandleBase> goal_handle);

  void handleMoveToPoseAccepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> goal_handle);
  void handleCheckPoseReachabilityAccepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<CheckPoseReachability>> goal_handle);
  void handleMoveToHomeAccepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToHome>> goal_handle);
  void handleLiftAccepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<LiftGripper>> goal_handle);

  bool reserveMotion();
  void launchWorker(std::function<void()> work);
  static bool isValidPose(const geometry_msgs::msg::Pose & pose);

  template<typename ActionT, typename WorkT>
  void executeGoal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    WorkT work,
    const std::string & state = "planning_and_executing")
  {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    feedback->state = state;
    goal_handle->publish_feedback(feedback);

    OperationResult outcome{false, "Motion failed unexpectedly"};
    try {
      outcome = work();
    } catch (const std::exception & error) {
      outcome.message = error.what();
      RCLCPP_ERROR(logger_, "Arm motion failed: %s", error.what());
    }

    auto result = std::make_shared<typename ActionT::Result>();
    result->success = outcome.success;
    result->message = outcome.message;
    // Release the server before publishing the terminal result so a client can
    // immediately submit the next candidate without racing this worker.
    motion_active_.store(false);
    if (outcome.success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  rclcpp::Logger logger_;
  MotionExecutor & motion_executor_;
  std::atomic_bool motion_active_{false};
  std::mutex workers_mutex_;
  std::vector<std::thread> workers_;
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_server_;
  rclcpp_action::Server<CheckPoseReachability>::SharedPtr check_pose_reachability_server_;
  rclcpp_action::Server<MoveToHome>::SharedPtr move_to_home_server_;
  rclcpp_action::Server<LiftGripper>::SharedPtr lift_server_;
};

}  // namespace arm

#endif  // ARM_BEHAVIOR__ARM_ACTION_SERVER_HPP_
