#include "rclcpp/rclcpp.hpp"

#include "arm_behavior/arm_behavior.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = arm::ArmBehavior::create();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
