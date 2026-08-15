#ifndef ARM_BEHAVIOR__MOTION_EXECUTOR_HPP_
#define ARM_BEHAVIOR__MOTION_EXECUTOR_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"

#include "arm_behavior/operation_result.hpp"

namespace arm
{

class MotionExecutor
{
public:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  MotionExecutor(
    std::shared_ptr<MoveGroupInterface> move_group,
    std::shared_ptr<std::mutex> moveit_mutex,
    std::string tcp_link,
    std::string home_pose_name);

  OperationResult moveToPose(const geometry_msgs::msg::Pose & target_pose);
  OperationResult moveToHomePose();
  OperationResult lift(double distance);

private:
  OperationResult planAndExecute(const std::string & motion_description);

  std::shared_ptr<MoveGroupInterface> move_group_;
  std::shared_ptr<std::mutex> moveit_mutex_;
  std::string tcp_link_;
  std::string home_pose_name_;
};

}  // namespace arm

#endif  // ARM_BEHAVIOR__MOTION_EXECUTOR_HPP_
