#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__OBSTACLE_CONTROL_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__OBSTACLE_CONTROL_PLUGIN_HPP_

#include <array>
#include <memory>

#include <mujoco_ros2_control_msgs/srv/set_obstacle.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/** @brief Keeps a free obstacle body at a service-controlled position. */
class ObstacleControlPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  ObstacleControlPlugin() = default;
  ~ObstacleControlPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using SetObstacle = mujoco_ros2_control_msgs::srv::SetObstacle;

  void apply_stored_pose(mjData* data) const;
  void handle_set_obstacle(const SetObstacle::Request::SharedPtr request, SetObstacle::Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("ObstacleControlPlugin") };
  rclcpp::Service<SetObstacle>::SharedPtr set_obstacle_service_;
  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  int qpos_address_{ -1 };
  int dof_address_{ -1 };
  std::array<double, 3> stored_position_{};
  std::array<double, 4> initial_orientation_{};
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__OBSTACLE_CONTROL_PLUGIN_HPP_
