/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mujoco_ros2_control/sensors/depth_to_pointcloud.hpp"
#include "mujoco_ros2_control/simulation/mujoco_simulation_clock.hpp"

namespace mujoco_ros2_control
{

struct CameraData
{
  mjvCamera mjv_cam;
  mjrRect viewport;

  std::string name;
  std::string frame_name;
  std::string pointcloud_topic;

  uint32_t width;
  uint32_t height;
  double fovy;

  std::vector<float> depth_buffer;
  std::vector<float> depth_buffer_flipped;
  std::unique_ptr<DepthToPointcloud> depth_to_pointcloud;

  sensor_msgs::msg::PointCloud2 pointcloud_message;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
  bool pointcloud_ready{ false };
  bool has_published{ false };
};

/**
 * @brief Wraps MuJoCo cameras for publishing ROS 2 point clouds.
 */
class MujocoCameras
{
public:
  /**
   * @brief Constructs a new MujocoCameras wrapper object.
   *
   * @param node Will be used to construct image publishers
   * @param simulation_clock Provides simulation-time scheduling for camera updates, and
   *        records when the cameras last rendered.
   * @param sim_mutex Provides synchronized access to the mujoco_data object for rendering
   * @param mujoco_data MuJoCo data for the simulation
   * @param mujoco_model MuJoCo model for the simulation
   * @param camera_publish_rate The rate to publish all camera images, for now all images are published at the same rate.
   */
  explicit MujocoCameras(rclcpp::Node::SharedPtr node, const MujocoSimulationClock& simulation_clock,
                         std::recursive_mutex* sim_mutex, mjData* mujoco_data, mjModel* mujoco_model,
                         double camera_publish_rate);

  /**
   * @brief Starts the image processing thread in the background.
   *
   * The background thread will initialize its own offscreen glfw context for rendering images that is
   * separate from the Simulate applications, as the context must be in the running thread.
   */
  void init();

  /**
   * @brief Stops the camera processing thread and closes the relevant objects, call before shutdown.
   */
  void close();

  /**
   * @brief Parses camera information from the mujoco model.
   */
  void register_cameras(const hardware_interface::HardwareInfo& hardware_info);

private:
  /**
   * @brief Extracts and validates the MuJoCo-specific parameters for one camera.
   *
   * @param camera_id Index into mjModel's camera arrays.
   * @return Camera data populated with name, fixed-camera ID, dimensions, viewport,
   *         and vertical field of view; std::nullopt when the camera is invalid.
   */
  std::optional<CameraData> extract_camera_parameters(int camera_id) const;

  /**
   * @brief Applies ROS topic and frame parameters for a camera, or logs and applies defaults.
   */
  void read_camera_hardware_parameters(CameraData& camera,
                                       const hardware_interface::HardwareInfo& hardware_info) const;

  /** @brief Allocates depth buffers and initializes the depth-to-point-cloud converter. */
  void initialize_depth_processing(CameraData& camera) const;

  /** @brief Initializes the PointCloud2 layout used to publish one camera's points. */
  void initialize_pointcloud_message(CameraData& camera) const;

  /** @brief Linearizes MuJoCo depth-buffer values and reverses their row order. */
  void linearize_and_flip_depth(CameraData& camera, float near, float depth_scale) const;

  /** @brief Copies an XYZ point cloud into its ROS PointCloud2 transport message. */
  void populate_pointcloud_message(CameraData& camera, const Pointcloud& pointcloud) const;

  /** @brief Initializes MuJoCo's camera-rendering visual options. */
  void initialize_visual_options();

  /** @brief Configures visual options so sensor aids are hidden from depth rendering. */
  void configure_depth_rendering_visibility();

  /** @brief Allocates the MuJoCo scene used for offscreen camera rendering. */
  void initialize_scene();

  /** @brief Initializes MuJoCo's offscreen renderer and sizes its framebuffer. */
  void initialize_renderer_context();

  /**
   * @brief Initializes the rendering context and starts processing.
   */
  void update_loop();

  /**
   * @brief Updates and publishes camera point clouds.
   */
  void update();

  rclcpp::Node::SharedPtr node_;
  const MujocoSimulationClock& simulation_clock_;

  // Ensures locked access to simulation data for rendering.
  std::recursive_mutex* sim_mutex_{ nullptr };

  mjData* mj_data_;
  mjModel* mj_model_;
  mjData* mj_camera_data_;

  // Simulation time of the snapshot rendered by the last update, also reported to the
  // simulation clock. The loop paces itself from this rather than from "now", so the next
  // update is due at a time the physics loop is allowed to reach.
  mjtNum last_update_sim_time_{ 0.0 };

  // Image publishing rate
  double camera_publish_rate_;

  // Rendering options for the cameras, currently hard coded to defaults
  mjvOption mjv_opt_;
  mjvScene mjv_scn_;
  mjrContext mjr_con_;

  // Containers for camera data and ROS constructs
  std::vector<CameraData> cameras_;

  // Camera processing thread
  std::thread rendering_thread_;
  // Start disabled.  This prevents the render loop from treating an
  // unsuccessful GLFW/context initialization as a running camera.
  std::atomic_bool publish_images_{ false };
};

}  // namespace mujoco_ros2_control
