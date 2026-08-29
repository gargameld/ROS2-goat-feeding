#ifndef ARM_BEHAVIOR__ARM_BEHAVIOR_HPP_
#define ARM_BEHAVIOR__ARM_BEHAVIOR_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm_behavior/arm_action_server.hpp"
#include "arm_behavior/motion_executor.hpp"
#include "arm_behavior/payload_service_server.hpp"
#include "arm_behavior/planning_scene_manager.hpp"

namespace arm
{

class ArmBehavior : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<ArmBehavior>;

  static SharedPtr create(rclcpp::NodeOptions options = rclcpp::NodeOptions());

private:
  struct Configuration
  {
    std::string planning_group;
    std::string tcp_link;
    std::string home_pose;
    std::string payload_id;
    std::array<double, 3> box_dimensions;
    std::vector<std::string> gripper_touch_links;
  };

  explicit ArmBehavior(const rclcpp::NodeOptions & options);

  void initialize();
  Configuration loadConfiguration();
  std::array<double, 3> readBoxDimensions();
  void initializeMoveIt(const Configuration & configuration);
  void initializeInterfaces(const Configuration & configuration);
  rclcpp::Node::SharedPtr nodeHandle();

  template<typename ParameterT>
  ParameterT getOrDeclare(const std::string & name, const ParameterT & default_value)
  {
    if (has_parameter(name)) {
      return get_parameter(name).get_value<ParameterT>();
    }
    return declare_parameter<ParameterT>(name, default_value);
  }

  static std::vector<std::string> defaultGripperTouchLinks();

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<std::mutex> moveit_mutex_;
  std::unique_ptr<PlanningSceneManager> planning_scene_manager_;
  std::unique_ptr<MotionExecutor> motion_executor_;
  std::unique_ptr<ArmActionServer> action_server_;
  std::unique_ptr<PayloadServiceServer> service_server_;
};

}  // namespace arm

#endif  // ARM_BEHAVIOR__ARM_BEHAVIOR_HPP_
