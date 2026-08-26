#include "arm_behavior/motion_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/conversions.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_behavior/simulation_control.hpp"

namespace
{

constexpr double kCartesianLiftStep = 0.01;
constexpr double kRequiredCartesianPathFraction = 0.999;
// Narrow passages (e.g. threading the arm between the 0.1m quoridor walls in
// environment_boxes.yaml) are low-probability regions for a randomized sampler,
// so planning here either succeeds almost immediately or not at all.
// Restarts are driven from here rather than through MoveIt's
// num_planning_attempts: those attempts run concurrently against a single
// shared allowed_planning_time, so they divide that budget instead of each
// getting one, and a failing request burns the whole budget no matter how many
// are requested. A sequential loop instead gives every retry a freshly seeded
// RRTConnect and the full per-retry time. Narrow-passage solutions have been
// measured arriving in ~1 s when they arrive at all, so a short per-retry
// budget costs almost nothing and buys many more independent restarts.
constexpr double kPlanningTimeSeconds = 20.0;
constexpr int kNumPlanningAttempts = 1;
constexpr int kPlanningRetries = 20;
constexpr double kReachabilityPlanningTimeSeconds = 7.0;
constexpr int kReachabilityPlanningAttempts = 1;
constexpr int kReachabilityPlanningRetries = 20;
// A goal pose whose only IK solutions are in collision cannot be planned to,
// but OMPL charges full price to find that out: RRTConnect blocks in
// nextGoal() until the planning-time termination condition fires, so an
// unreachable candidate costs kReachabilityPlanningTimeSeconds *
// kReachabilityPlanningRetries no matter how hopeless it is. /compute_ik with
// avoid_collisions runs exactly the check that fails inside OMPL's goal
// sampler -- collision-aware IK against the monitored planning scene -- once,
// up front, where the answer is visible. KDL keeps re-seeding randomly until
// this budget is gone, so it must be long enough for a genuinely reachable
// pose to converge, not merely for one restart.
constexpr double kGoalIkTimeoutSeconds = 1.0;
// Wall-clock allowance for the round trip, which also covers move_group
// locking the planning scene while a plan is in flight.
constexpr std::chrono::seconds kComputeIkServiceTimeout{5};
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
    case ErrorCodes::FAILURE:
      return "FAILURE (the planner gave up without finding a path)";
    default:
      return "unclassified MoveIt error";
  }
}

// Plan repeatedly until one attempt succeeds, returning that attempt's code.
// Each call to plan() reseeds the planner, so the retries are independent in a
// way MoveIt's own concurrent attempts are not (see the retry constants above).
moveit::core::MoveItErrorCode plan_with_retries(
  moveit::planning_interface::MoveGroupInterface & move_group,
  int retries,
  const std::string & motion_description,
  moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  const auto logger = rclcpp::get_logger("arm.motion_executor");
  moveit::core::MoveItErrorCode planning_result(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  for (int retry = 1; retry <= retries; ++retry) {
    planning_result = move_group.plan(plan);
    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(
        logger, "Planning for %s succeeded on retry %d of %d",
        motion_description.c_str(), retry, retries);
      return planning_result;
    }
    RCLCPP_INFO(
      logger, "Planning retry %d of %d for %s failed: code=%d (%s)",
      retry, retries, motion_description.c_str(), planning_result.val,
      describe_moveit_error(planning_result).c_str());
  }
  return planning_result;
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
  compute_ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
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
  // Reachability is only a fast screening pass over many ranked candidates.
  // Keep full planning budgets for actual arm motions, but do not spend them
  // repeatedly on grasp poses that have no valid goal state.
  move_group_->setPlanningTime(kReachabilityPlanningTimeSeconds);
  move_group_->setNumPlanningAttempts(kReachabilityPlanningAttempts);
  std::string target_frame;
  auto result = setPoseTarget(target_pose, reference_frame, target_frame);
  const std::string motion_description = "target pose in frame '" + target_frame + "'";
  if (result.success) {
    // Screen out goal poses with no collision-free IK solution before OMPL
    // spends the whole retry budget rediscovering it one goal sample at a time.
    geometry_msgs::msg::PoseStamped stamped_target;
    stamped_target.header.frame_id = target_frame;
    stamped_target.pose = target_pose;
    result = checkGoalStateValidity(stamped_target, motion_description);
  }
  if (result.success) {
    result = planOnly(motion_description);
  }
  move_group_->setPlanningTime(kPlanningTimeSeconds);
  move_group_->setNumPlanningAttempts(kNumPlanningAttempts);
  return result;
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
    // Collision checking is disabled for the retreat. The gripper has just
    // closed on the food cube, so the fingers are in contact with it by
    // definition and the octomap still holds the voxels they closed around;
    // a collision-checked lift rejects its very first waypoint. The motion is
    // a short, straight, vertical retreat out of a shelf whose surroundings
    // were already checked on the way in.
    fraction = move_group_->computeCartesianPath(
      {target_pose}, kCartesianLiftStep, trajectory, false);
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
    return {false, "Unable to compute a complete Cartesian lift"};
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
    planning_result = plan_with_retries(
      *move_group_, kPlanningRetries, motion_description, plan);
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
    planning_result = plan_with_retries(
      *move_group_, kReachabilityPlanningRetries, motion_description, plan);
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

OperationResult MotionExecutor::checkGoalStateValidity(
  const geometry_msgs::msg::PoseStamped & target_pose,
  const std::string & motion_description)
{
  const auto logger = rclcpp::get_logger("arm.motion_executor");
  // A missing or slow service must not silently reject reachable grasps: fall
  // through to planning, which is the authority either way.
  if (!compute_ik_client_->wait_for_service(kComputeIkServiceTimeout)) {
    RCLCPP_WARN(
      logger, "'compute_ik' is unavailable; planning %s without a goal-state pre-check",
      motion_description.c_str());
    return {true, "Goal-state pre-check skipped"};
  }

  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  auto & ik_request = request->ik_request;
  ik_request.group_name = move_group_->getName();
  ik_request.ik_link_name = tcp_link_;
  ik_request.pose_stamped = target_pose;
  ik_request.avoid_collisions = true;
  ik_request.timeout = rclcpp::Duration::from_seconds(kGoalIkTimeoutSeconds);
  // Seed from the state the plan would start in, so the pre-check and the
  // planner agree on which IK branches are within reach.
  if (const auto current_state = move_group_->getCurrentState()) {
    moveit::core::robotStateToRobotStateMsg(*current_state, ik_request.robot_state);
  }

  auto future = compute_ik_client_->async_send_request(request);
  if (future.wait_for(kComputeIkServiceTimeout) != std::future_status::ready) {
    compute_ik_client_->remove_pending_request(future);
    RCLCPP_WARN(
      logger, "'compute_ik' did not answer within %ld s; planning %s without a goal-state pre-check",
      static_cast<long>(kComputeIkServiceTimeout.count()), motion_description.c_str());
    return {true, "Goal-state pre-check skipped"};
  }

  const moveit::core::MoveItErrorCode ik_result(future.get()->error_code);
  if (ik_result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(
      logger, "Goal state for %s has a collision-free IK solution", motion_description.c_str());
    return {true, "Goal state is valid"};
  }

  const auto error_description = describe_moveit_error(ik_result);
  RCLCPP_INFO(
    logger,
    "No collision-free IK solution for %s within %.3f s: code=%d (%s); skipping planning",
    motion_description.c_str(), kGoalIkTimeoutSeconds, ik_result.val, error_description.c_str());
  move_group_->clearPoseTargets();
  return {
    false,
    "No collision-free IK solution for " + motion_description + "; compute_ik code=" +
    std::to_string(ik_result.val) + " (" + error_description + ")"};
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
