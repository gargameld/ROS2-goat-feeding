#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__LOCK_ROBOT_BASE_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__LOCK_ROBOT_BASE_PLUGIN_HPP_

#include <array>
#include <atomic>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Pins the free-floating robot base to the pose it had when the lock service was called.
 *
 * The base body is expected to carry a free joint (7 qpos values, 6 dofs). Locking snapshots
 * those 7 qpos values; every update() then writes the snapshot back and zeroes the base
 * velocity and acceleration, so the chassis stays put while the arm keeps moving. Unlocking
 * stops the rewriting and lets the base fall back under physics control.
 */
class LockRobotBasePlugin : public MuJoCoROS2ControlPluginBase
{
public:
  LockRobotBasePlugin() = default;
  ~LockRobotBasePlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using Trigger = std_srvs::srv::Trigger;

  void handle_lock_base(const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  void handle_unlock_base(const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);

  /// Copy the current free-joint pose out of the physics data into locked_pose_.
  void snapshot_pose();

  /// Overwrite the free-joint pose with locked_pose_ and zero its velocity and acceleration.
  void apply_locked_pose(mjData* data) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("LockRobotBasePlugin") };
  rclcpp::Service<Trigger>::SharedPtr lock_base_service_;
  rclcpp::Service<Trigger>::SharedPtr unlock_base_service_;

  std::string body_name_;

  // The physics mjData handed to init(). update() receives the controller-facing copy, which the
  // physics loop overwrites after every step, so the pose has to be written here to take effect.
  mjData* physics_data_{ nullptr };

  // Indices of the base free joint within qpos / qvel, resolved once during init().
  int qpos_address_{ -1 };
  int dof_address_{ -1 };

  std::atomic<bool> locked_{ false };
  std::array<mjtNum, 7> locked_pose_{};
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__LOCK_ROBOT_BASE_PLUGIN_HPP_
