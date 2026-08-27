#include "arm_behavior/arm_behavior.hpp"

#include <stdexcept>
#include <utility>

namespace arm
{

ArmBehavior::SharedPtr ArmBehavior::create(rclcpp::NodeOptions options)
{
  options.automatically_declare_parameters_from_overrides(true);
  auto node = SharedPtr(new ArmBehavior(options));
  node->initialize();
  return node;
}

ArmBehavior::ArmBehavior(const rclcpp::NodeOptions & options)
: Node("arm_behavior", options)
{
}

void ArmBehavior::initialize()
{
  const auto configuration = loadConfiguration();
  initializeMoveIt(configuration);
  initializeInterfaces(configuration);
  RCLCPP_INFO(
    get_logger(), "Arm actions and payload services are ready in namespace %s",
    get_namespace());
}

ArmBehavior::Configuration ArmBehavior::loadConfiguration()
{
  return {
    getOrDeclare<std::string>("planning_group", "arm_planning_group"),
    getOrDeclare<std::string>("tcp_link", "arm_tcp"),
    getOrDeclare<std::string>("home_pose", "home_pose"),
    getOrDeclare<std::string>("payload.id", "lifted_payload"),
    readBoxDimensions(),
    getOrDeclare<std::vector<std::string>>(
      "payload.gripper_touch_links", defaultGripperTouchLinks())
  };
}

std::array<double, 3> ArmBehavior::readBoxDimensions()
{
  const auto dimensions = getOrDeclare<std::vector<double>>(
    "payload.box_dimensions", {0.10, 0.10, 0.15});
  if (dimensions.size() != 3U) {
    throw std::runtime_error("payload.box_dimensions must contain [x, y, z]");
  }
  for (const double dimension : dimensions) {
    if (dimension <= 0.0) {
      throw std::runtime_error("Every payload box dimension must be positive");
    }
  }
  return {dimensions[0], dimensions[1], dimensions[2]};
}

void ArmBehavior::initializeMoveIt(const Configuration & configuration)
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    nodeHandle(), configuration.planning_group);
  move_group_->setEndEffectorLink(configuration.tcp_link);
  moveit_mutex_ = std::make_shared<std::mutex>();
}

void ArmBehavior::initializeInterfaces(const Configuration & configuration)
{
  motion_executor_ = std::make_unique<MotionExecutor>(
    nodeHandle(), move_group_, moveit_mutex_, configuration.tcp_link, configuration.home_pose);
  payload_manager_ = std::make_unique<PayloadManager>(
    moveit_mutex_, configuration.tcp_link, configuration.payload_id,
    configuration.box_dimensions, configuration.gripper_touch_links);
  action_server_ = std::make_unique<ArmActionServer>(nodeHandle(), *motion_executor_);
  service_server_ = std::make_unique<PayloadServiceServer>(nodeHandle(), *payload_manager_);
}

rclcpp::Node::SharedPtr ArmBehavior::nodeHandle()
{
  // Components are owned by this node, so their node handle must not own the
  // node in return. The factory guarantees that initialization happens only
  // after ArmBehavior itself has a real shared owner.
  return rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
}

std::vector<std::string> ArmBehavior::defaultGripperTouchLinks()
{
  return {
    "arm_tcp",
    "gripper_base_link",
    "left_jaw_link",
    "right_jaw_link"
  };
}

}  // namespace arm
