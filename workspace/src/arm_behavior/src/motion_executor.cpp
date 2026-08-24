#include "arm_behavior/motion_executor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_behavior/simulation_control.hpp"

namespace
{

constexpr double kCartesianLiftStep = 0.01;
constexpr double kRequiredCartesianPathFraction = 0.999;
// Narrow passages (e.g. threading the arm between the 0.1m quoridor walls in
// environment_boxes.yaml) are low-probability regions for a randomized sampler.
// Give OMPL a larger time budget and, crucially, several independent attempts:
// each attempt reseeds RRTConnect's trees, so N attempts of T seconds is far
// more likely to find a path than one attempt of N*T seconds.
constexpr double kPlanningTimeSeconds = 15.0;
constexpr int kNumPlanningAttempts = 10;
constexpr double kGoalPositionToleranceMeters = 0.02;
constexpr double kGoalOrientationToleranceRadians = 0.0872665;
constexpr double kGoalJointToleranceRadians = 0.01;

void log_pose(
  const rclcpp::Logger & logger,
  const std::string & label,
  const geometry_msgs::msg::Pose & pose)
{
  RCLCPP_INFO(
    logger,
    "%s position=[%.6f, %.6f, %.6f] orientation=[%.6f, %.6f, %.6f, %.6f]",
    label.c_str(), pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

std::string format_positions(const std::vector<double> & positions)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << '[';
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (index > 0U) {
      stream << ", ";
    }
    stream << positions[index];
  }
  stream << ']';
  return stream.str();
}

std::string format_joint_state(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << '[';
  const std::size_t count = std::min(joint_names.size(), positions.size());
  for (std::size_t index = 0; index < count; ++index) {
    if (index > 0U) {
      stream << ", ";
    }
    stream << joint_names[index] << '=' << positions[index];
  }
  if (joint_names.size() != positions.size()) {
    stream << "; names=" << joint_names.size() << ", positions=" << positions.size();
  }
  stream << ']';
  return stream.str();
}

double quaternion_norm(const geometry_msgs::msg::Pose & pose)
{
  const auto & orientation = pose.orientation;
  return std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
}

std::string describe_moveit_error(const moveit::core::MoveItErrorCode & error)
{
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  switch (error.val) {
    case ErrorCodes::SUCCESS:
      return "SUCCESS";
    case ErrorCodes::PLANNING_FAILED:
      return "PLANNING_FAILED (no valid path was found)";
    case ErrorCodes::INVALID_MOTION_PLAN:
      return "INVALID_MOTION_PLAN";
    case ErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case ErrorCodes::TIMED_OUT:
      return "TIMED_OUT";
    case ErrorCodes::START_STATE_IN_COLLISION:
      return "START_STATE_IN_COLLISION";
    case ErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case ErrorCodes::START_STATE_INVALID:
      return "START_STATE_INVALID";
    case ErrorCodes::GOAL_IN_COLLISION:
      return "GOAL_IN_COLLISION";
    case ErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
      return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case ErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
      return "GOAL_CONSTRAINTS_VIOLATED";
    case ErrorCodes::GOAL_STATE_INVALID:
      return "GOAL_STATE_INVALID";
    case ErrorCodes::FRAME_TRANSFORM_FAILURE:
      return "FRAME_TRANSFORM_FAILURE";
    case ErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
      return "COLLISION_CHECKING_UNAVAILABLE";
    case ErrorCodes::ROBOT_STATE_STALE:
      return "ROBOT_STATE_STALE";
    case ErrorCodes::NO_IK_SOLUTION:
      return "NO_IK_SOLUTION";
    default:
      return "unclassified MoveIt error";
  }
}

}  // namespace

namespace arm
{

MotionExecutor::MotionExecutor(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<MoveGroupInterface> move_group,
  std::shared_ptr<std::mutex> moveit_mutex,
  std::string tcp_link,
  std::string home_pose_name)
: move_group_(std::move(move_group)),
  node_(std::move(node)),
  moveit_mutex_(std::move(moveit_mutex)),
  tcp_link_(std::move(tcp_link)),
  home_pose_name_(std::move(home_pose_name))
{
  move_group_->setPlanningTime(kPlanningTimeSeconds);
  move_group_->setNumPlanningAttempts(kNumPlanningAttempts);
  move_group_->setGoalPositionTolerance(kGoalPositionToleranceMeters);
  move_group_->setGoalOrientationTolerance(kGoalOrientationToleranceRadians);
  move_group_->setGoalJointTolerance(kGoalJointToleranceRadians);
}

OperationResult MotionExecutor::moveToPose(
  const geometry_msgs::msg::Pose & target_pose,
  const std::string & reference_frame)
{
  std::lock_guard<std::mutex> lock(*moveit_mutex_);
  std::string target_frame;
  const auto target_result = setPoseTarget(target_pose, reference_frame, target_frame);
  if (!target_result.success) {
    return target_result;
  }
  return planAndExecute("target pose in frame '" + target_frame + "'");
}

OperationResult MotionExecutor::checkPoseReachability(
  const geometry_msgs::msg::Pose & target_pose,
  const std::string & reference_frame)
{
  std::lock_guard<std::mutex> lock(*moveit_mutex_);
  std::string target_frame;
  const auto target_result = setPoseTarget(target_pose, reference_frame, target_frame);
  if (!target_result.success) {
    return target_result;
  }
  return planOnly("target pose in frame '" + target_frame + "'");
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
  const auto logger = rclcpp::get_logger("arm.motion_executor");
  log_pose(logger, "Lift source pose for " + tcp_link_, target_pose);
  target_pose.position.z += distance;
  RCLCPP_INFO(logger, "Requested lift distance: %.6f m", distance);
  log_pose(logger, "Lift target pose for " + tcp_link_, target_pose);

  move_group_->setStartStateToCurrentState();
  move_group_->clearPoseTargets();
  if (!move_group_->setPoseTarget(target_pose, tcp_link_)) {
    return {false, "MoveIt rejected the lifted arm_tcp target pose"};
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  pause_simulation(node_);
  double fraction;
  try {
    fraction = move_group_->computeCartesianPath(
      {target_pose}, kCartesianLiftStep, trajectory, true);
  } catch (...) {
    try {
      resume_simulation(node_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        logger, "Failed to resume simulation after Cartesian planning error: %s", error.what());
    }
    throw;
  }
  resume_simulation(node_);
  move_group_->clearPoseTargets();
  if (fraction < kRequiredCartesianPathFraction) {
    return {false, "Unable to compute a complete collision-free Cartesian lift"};
  }

  MoveGroupInterface::Plan plan;
  plan.trajectory = std::move(trajectory);
  log_path(plan, "Cartesian lift");
  const auto execution_result = move_group_->execute(plan);
  if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
    return {false, "Failed to execute Cartesian lift"};
  }
  return {true, "Cartesian lift completed"};
}

void MotionExecutor::log_path(
  const MoveGroupInterface::Plan & plan,
  const std::string & motion_description) const
{
  const auto logger = rclcpp::get_logger("arm.motion_executor");
  const auto & trajectory = plan.trajectory.joint_trajectory;
  RCLCPP_INFO(
    logger, "Planned %s path: planning_time=%.6f s, joints=%zu, waypoints=%zu",
    motion_description.c_str(), plan.planning_time, trajectory.joint_names.size(),
    trajectory.points.size());

  std::ostringstream joint_names;
  for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
    if (index > 0U) {
      joint_names << ", ";
    }
    joint_names << trajectory.joint_names[index];
  }
  RCLCPP_INFO(logger, "Path joint order: [%s]", joint_names.str().c_str());

  if (trajectory.points.empty()) {
    RCLCPP_WARN(logger, "Planned path contains no joint trajectory points");
    return;
  }

  const auto & first = trajectory.points.front();
  const auto & last = trajectory.points.back();
  RCLCPP_INFO(
    logger, "Path start joints=%s", format_positions(first.positions).c_str());
  RCLCPP_INFO(
    logger,
    "Path end joints=%s at %d.%09u s",
    format_positions(last.positions).c_str(), last.time_from_start.sec,
    last.time_from_start.nanosec);
}

void MotionExecutor::log_planning_context(const std::string & motion_description) const
{
  const auto logger = rclcpp::get_logger("arm.motion_executor");
  RCLCPP_INFO(
    logger,
    "Planning diagnostics for %s: planner_id='%s', planning_time=%.3f s, "
    "goal_tolerances=[joint=%.6f rad, position=%.6f m, orientation=%.6f rad]",
    motion_description.c_str(), move_group_->getPlannerId().c_str(),
    move_group_->getPlanningTime(), move_group_->getGoalJointTolerance(),
    move_group_->getGoalPositionTolerance(), move_group_->getGoalOrientationTolerance());
  RCLCPP_INFO(
    logger, "Pose reference frame='%s', end effector='%s', planning group joints=%s",
    move_group_->getPoseReferenceFrame().c_str(), tcp_link_.c_str(),
    format_joint_state(move_group_->getJointNames(), move_group_->getCurrentJointValues()).c_str());

  const auto current_tcp_pose = move_group_->getCurrentPose(tcp_link_);
  RCLCPP_INFO(logger, "Current arm_tcp pose frame='%s'", current_tcp_pose.header.frame_id.c_str());
  log_pose(logger, "Current arm_tcp pose", current_tcp_pose.pose);
}

OperationResult MotionExecutor::planAndExecute(const std::string & motion_description)
{
  log_planning_context(motion_description);
  MoveGroupInterface::Plan plan;
  pause_simulation(node_);
  moveit::core::MoveItErrorCode planning_result;
  try {
    planning_result = move_group_->plan(plan);
  } catch (...) {
    try {
      resume_simulation(node_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        rclcpp::get_logger("arm.motion_executor"),
        "Failed to resume simulation after planning error: %s", error.what());
    }
    throw;
  }
  resume_simulation(node_);
  if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
    const auto error_description = describe_moveit_error(planning_result);
    const auto logger = rclcpp::get_logger("arm.motion_executor");
    RCLCPP_ERROR(
      logger, "Planning failed for %s: code=%d (%s), source='%s', message='%s'",
      motion_description.c_str(), planning_result.val, error_description.c_str(),
      planning_result.source.c_str(), planning_result.message.c_str());
    move_group_->clearPoseTargets();
    return {
      false,
      "Failed to plan arm motion to " + motion_description + "; MoveIt code=" +
      std::to_string(planning_result.val) + " (" + error_description + ")"
    };
  }

  log_path(plan, motion_description);
  const auto execution_result = move_group_->execute(plan);
  move_group_->clearPoseTargets();
  if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
    const auto error_description = describe_moveit_error(execution_result);
    RCLCPP_ERROR(
      rclcpp::get_logger("arm.motion_executor"),
      "Execution failed for %s: code=%d (%s), source='%s', message='%s'",
      motion_description.c_str(), execution_result.val, error_description.c_str(),
      execution_result.source.c_str(), execution_result.message.c_str());
    return {
      false,
      "Failed to execute arm motion to " + motion_description + "; MoveIt code=" +
      std::to_string(execution_result.val) + " (" + error_description + ")"
    };
  }
  return {true, "Arm motion to " + motion_description + " completed"};
}

OperationResult MotionExecutor::planOnly(const std::string & motion_description)
{
  log_planning_context(motion_description);
  MoveGroupInterface::Plan plan;
  pause_simulation(node_);
  moveit::core::MoveItErrorCode planning_result;
  try {
    planning_result = move_group_->plan(plan);
  } catch (...) {
    try {
      resume_simulation(node_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        rclcpp::get_logger("arm.motion_executor"),
        "Failed to resume simulation after planning error: %s", error.what());
    }
    throw;
  }
  resume_simulation(node_);
  if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
    const auto error_description = describe_moveit_error(planning_result);
    RCLCPP_INFO(
      rclcpp::get_logger("arm.motion_executor"),
      "Pose is not reachable for %s: code=%d (%s)", motion_description.c_str(),
      planning_result.val, error_description.c_str());
    move_group_->clearPoseTargets();
    return {
      false,
      "Failed to plan arm motion to " + motion_description + "; MoveIt code=" +
      std::to_string(planning_result.val) + " (" + error_description + ")"
    };
  }
  log_path(plan, motion_description);
  move_group_->clearPoseTargets();
  return {true, "Arm pose is reachable: " + motion_description};
}

OperationResult MotionExecutor::setPoseTarget(
  const geometry_msgs::msg::Pose & target_pose,
  const std::string & reference_frame,
  std::string & target_frame)
{
  move_group_->setStartStateToCurrentState();
  move_group_->clearPoseTargets();
  geometry_msgs::msg::PoseStamped stamped_target;
  target_frame = reference_frame.empty() ? move_group_->getPlanningFrame() : reference_frame;
  stamped_target.header.frame_id = target_frame;
  stamped_target.pose = target_pose;
  const auto logger = rclcpp::get_logger("arm.motion_executor");
  log_pose(logger, "Requested arm_tcp target", target_pose);
  const double target_quaternion_norm = quaternion_norm(target_pose);
  RCLCPP_INFO(
    logger, "Target frame='%s', planning frame='%s', target quaternion norm=%.9f",
    target_frame.c_str(), move_group_->getPlanningFrame().c_str(), target_quaternion_norm);
  if (std::abs(target_quaternion_norm - 1.0) > 1e-3) {
    RCLCPP_WARN(
      logger, "Target quaternion is not normalized (norm=%.9f); this can prevent IK/planning",
      target_quaternion_norm);
  }
  if (!move_group_->setPoseTarget(stamped_target, tcp_link_)) {
    RCLCPP_ERROR(logger, "MoveIt rejected the pose target before planning");
    return {false, "MoveIt rejected the arm_tcp target pose in frame '" + target_frame + "'"};
  }
  return {true, "Pose target accepted"};
}

}  // namespace arm
