
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <mujoco/mujoco.h>

#include "mujoco_ros2_control/data.hpp"
#include "mujoco_ros2_control/sensors/mujoco_cameras.hpp"
#include "mujoco_ros2_control/simulation/mujoco_simulation.hpp"
#include "mujoco_ros2_control/simulation/physics_loop_synchronizer.hpp"

#include "mujoco_ros2_control/system_interface/component_info_map.hpp"
#include "mujoco_ros2_control/system_interface/control_plugin_loader.hpp"

namespace mujoco_ros2_control
{
class MujocoSystemInterface : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief ros2_control SystemInterface to wrap Mujocos Simulate application.
   *
   * Supports Actuators, IMU sensors, and RGB-D cameras in ROS 2 simulations.
   * For more information on configuration refer to the docs and the comment strings below.
   */
  MujocoSystemInterface();
  ~MujocoSystemInterface() override;

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  rclcpp::Logger get_logger() const;

private:
  /// Get the node of the MuJoCoSystemInterface.
  /**
   * \return node of the MuJoCoSystemInterface.
   */
  rclcpp::Node::SharedPtr get_node() const;

  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("MujocoSystemInterface");

  // Declared before simulation_ so the simulation and its physics thread are
  // destroyed first.
  std::unique_ptr<PhysicsLoopSynchronizer> physics_loop_synchronizer_;

  // The simulation host: owns the Simulate app, model/data, physics & UI threads,
  // clock publisher, and reset/pause/step services.
  std::unique_ptr<MujocoSimulation> simulation_;

  // Updated by write() and observed by the physics thread. The negative
  // sentinel prevents physics from advancing before the first write.
  rclcpp::Time last_ros_write_time_{ 0, 0, RCL_ROS_TIME };
  std::mutex last_ros_write_time_mutex_;

  // Provides access to ROS interfaces for elements that require it
  std::shared_ptr<rclcpp::Node> mujoco_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;

  // Containers for RGB-D cameras
  std::unique_ptr<MujocoCameras> cameras_;

  // Data containers for the HW interface
  ComponentInfoMap joint_hw_info_;
  ComponentInfoMap sensors_hw_info_;

  // Data containers for the MuJoCo Actuators
  std::vector<MuJoCoActuatorData> mujoco_actuator_data_;

  // Data containers for the URDF joints
  std::vector<URDFJointData> urdf_joint_data_;

  // ros2_control plugins configured for this simulation
  ControlPluginLoader plugin_loader_;

  std::vector<IMUSensorData> imu_sensor_data_;
};

}  // namespace mujoco_ros2_control
