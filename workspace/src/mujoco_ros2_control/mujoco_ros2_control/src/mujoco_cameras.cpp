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

#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/utils.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <exception>
#include <limits>
#include <utility>

using namespace std::chrono_literals;

namespace mujoco_ros2_control
{

MujocoCameras::MujocoCameras(rclcpp::Node::SharedPtr node, const MujocoSimulation& simulation,
                             std::recursive_mutex* sim_mutex, mjData* mujoco_data, mjModel* mujoco_model,
                             double camera_publish_rate)
  : node_(node)
  , simulation_clock_(simulation)
  , sim_mutex_(sim_mutex)
  , mj_data_(mujoco_data)
  , mj_model_(mujoco_model)
  , camera_publish_rate_(camera_publish_rate)
{
}

std::optional<CameraData> MujocoCameras::extract_camera_parameters(int camera_id) const
{
  if (camera_id < 0 || camera_id >= mj_model_->ncam)
  {
    RCLCPP_ERROR(node_->get_logger(), "Camera ID %d is outside the MuJoCo camera range.", camera_id);
    return std::nullopt;
  }

  const char* camera_name = mj_model_->names + mj_model_->name_camadr[camera_id];
  const int* camera_resolution = mj_model_->cam_resolution + 2 * camera_id;
  if (camera_resolution[0] <= 0 || camera_resolution[1] <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Camera '%s' has invalid resolution %d x %d; skipping it.", camera_name,
                 camera_resolution[0], camera_resolution[1]);
    return std::nullopt;
  }

  CameraData camera;
  camera.name = camera_name;
  camera.mjv_cam.type = mjCAMERA_FIXED;
  camera.mjv_cam.fixedcamid = camera_id;
  camera.width = static_cast<uint32_t>(camera_resolution[0]);
  camera.height = static_cast<uint32_t>(camera_resolution[1]);
  camera.fovy = mj_model_->cam_fovy[camera_id];
  camera.viewport = { 0, 0, camera_resolution[0], camera_resolution[1] };
  return camera;
}

void MujocoCameras::read_camera_hardware_parameters(CameraData& camera,
                                                    const hardware_interface::HardwareInfo& hardware_info) const
{
  const auto camera_info_maybe = get_sensor_from_info(hardware_info, camera.name);
  if (!camera_info_maybe.has_value())
  {
    camera.frame_name = camera.name + "_frame";
    camera.pointcloud_topic = camera.name + "/points";
    RCLCPP_INFO(node_->get_logger(), "No ros2_control sensor configuration found for camera '%s'; using default frame "
                                      "and point-cloud topic.",
                camera.name.c_str());
    return;
  }

  const auto& camera_info = camera_info_maybe.value();
  camera.frame_name = camera_info.parameters.at("frame_name");
  const auto pointcloud_topic = camera_info.parameters.find("pointcloud_topic");
  if (pointcloud_topic != camera_info.parameters.end())
  {
    camera.pointcloud_topic = pointcloud_topic->second;
    return;
  }

  const auto depth_topic = camera_info.parameters.find("depth_topic");
  if (depth_topic != camera_info.parameters.end())
  {
    camera.pointcloud_topic = depth_topic->second;
    RCLCPP_WARN(node_->get_logger(), "Camera '%s' uses deprecated parameter 'depth_topic'; use 'pointcloud_topic' "
                                      "instead.",
                camera.name.c_str());
    return;
  }

  camera.pointcloud_topic = camera.name + "/points";
  RCLCPP_WARN(node_->get_logger(), "Camera '%s' has no pointcloud_topic; using '%s'.", camera.name.c_str(),
              camera.pointcloud_topic.c_str());
}

void MujocoCameras::initialize_depth_processing(CameraData& camera) const
{
  camera.depth_buffer.resize(camera.width * camera.height);
  camera.depth_buffer_flipped.resize(camera.width * camera.height);
  camera.depth_to_pointcloud = std::make_unique<DepthToPointcloud>(camera.width, camera.height, camera.fovy);
}

void MujocoCameras::initialize_pointcloud_message(CameraData& camera) const
{
  auto& message = camera.pointcloud_message;
  message.header.frame_id = camera.frame_name;
  message.height = camera.height;
  message.width = camera.width;
  message.is_bigendian = false;
  // Pixels whose ray hits nothing (open space beyond any geometry) are marked
  // NaN by linearize_and_flip_depth rather than reporting the far clipping
  // plane as a real surface, so the cloud is not dense.
  message.is_dense = false;
  message.point_step = 3U * static_cast<uint32_t>(sizeof(float));
  message.row_step = message.width * message.point_step;
  sensor_msgs::msg::PointField x_field;
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;

  sensor_msgs::msg::PointField y_field = x_field;
  y_field.name = "y";
  y_field.offset = static_cast<uint32_t>(sizeof(float));

  sensor_msgs::msg::PointField z_field = x_field;
  z_field.name = "z";
  z_field.offset = 2U * static_cast<uint32_t>(sizeof(float));

  message.fields = { x_field, y_field, z_field };
  message.data.resize(static_cast<size_t>(message.row_step) * message.height);
}

void MujocoCameras::linearize_and_flip_depth(CameraData& camera, float near, float depth_scale) const
{
  // The scene's ground plane is 200m across and there is no explicit
  // <statistic extent="..."/>, so MuJoCo auto-derives near/far from that
  // plane's bounding box: far ends up thousands of metres out. Grazing rays
  // near the horizon legitimately intersect that distant floor, so their raw
  // depth is not exactly 1 (the literal clip value) -- it is just very close
  // to it -- yet 1/(1-depth) still blows the linearized distance up to
  // hundreds or thousands of metres. Those returns are technically real
  // intersections but are physically useless for this camera's tabletop-scale
  // task, so anything past a sane working range is treated as invalid rather
  // than polluting the cloud with kilometre-scale outliers.
  constexpr float kMaxValidDepthMetres = 10.0f;
  for (uint32_t row = 0; row < camera.height; ++row)
  {
    for (uint32_t column = 0; column < camera.width; ++column)
    {
      const auto index = row * camera.width + column;
      const auto flipped_index = (camera.height - 1 - row) * camera.width + column;
      const float raw_depth = camera.depth_buffer[index];
      const float linear_depth = near / (1.0f - raw_depth * depth_scale);
      camera.depth_buffer_flipped[flipped_index] =
          (linear_depth > kMaxValidDepthMetres) ? std::numeric_limits<float>::quiet_NaN() : linear_depth;
    }
  }
}

void MujocoCameras::populate_pointcloud_message(CameraData& camera, const Pointcloud& pointcloud) const
{
  sensor_msgs::PointCloud2Iterator<float> x(camera.pointcloud_message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(camera.pointcloud_message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(camera.pointcloud_message, "z");
  for (const auto& point : pointcloud)
  {
    *x = point.x;
    *y = point.y;
    *z = point.z;
    ++x;
    ++y;
    ++z;
  }
}

void MujocoCameras::initialize_visual_options()
{
  mjv_defaultOption(&mjv_opt_);
}

void MujocoCameras::configure_depth_rendering_visibility()
{
  // Do not render rangefinder rays or site markers into the depth images.
  mjv_opt_.flags[mjtVisFlag::mjVIS_RANGEFINDER] = 0;
  for (int i = 0; i < mjNGROUP; i++)
  {
    mjv_opt_.sitegroup[i] = 0;
  }
}

void MujocoCameras::initialize_scene()
{
  mjv_defaultScene(&mjv_scn_);
  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
}

void MujocoCameras::initialize_renderer_context()
{
  mjr_defaultContext(&mjr_con_);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);

  int max_width = 1;
  int max_height = 1;
  for (const auto& camera : cameras_)
  {
    max_width = std::max(max_width, static_cast<int>(camera.width));
    max_height = std::max(max_height, static_cast<int>(camera.height));
  }
  mjr_resizeOffscreen(max_width, max_height, &mjr_con_);
  RCLCPP_INFO(node_->get_logger(), "Resized offscreen buffer to %d x %d", max_width, max_height);
}

void MujocoCameras::register_cameras(const hardware_interface::HardwareInfo& hardware_info)
{
  cameras_.resize(0);
  for (auto i = 0; i < mj_model_->ncam; ++i)
  {
    auto camera_maybe = extract_camera_parameters(i);
    if (!camera_maybe.has_value())
    {
      continue;
    }
    auto camera = std::move(camera_maybe.value());

    read_camera_hardware_parameters(camera, hardware_info);

    RCLCPP_INFO(node_->get_logger(), "Adding camera: '%s'", camera.name.c_str());
    RCLCPP_INFO(node_->get_logger(), "    frame_name: '%s'", camera.frame_name.c_str());
    RCLCPP_INFO(node_->get_logger(), "    pointcloud_topic: '%s'", camera.pointcloud_topic.c_str());

    // Use a reliable QoS profile for camera point clouds.
    const auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    camera.pointcloud_pub =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(camera.pointcloud_topic, camera_qos);

    initialize_depth_processing(camera);
    initialize_pointcloud_message(camera);
    RCLCPP_INFO(node_->get_logger(), "Camera '%s' PointCloud2 publisher is ready on '%s' (%u x %u).",
                camera.name.c_str(), camera.pointcloud_topic.c_str(), camera.width, camera.height);

    // Add to list of cameras
    cameras_.push_back(std::move(camera));
  }
}

void MujocoCameras::init()
{
  RCLCPP_INFO(node_->get_logger(), "Starting camera publishing for %zu registered camera(s).", cameras_.size());
  // Start the rendering thread process
  if (!glfwInit())
  {
    RCLCPP_WARN(node_->get_logger(), "Failed to initialize GLFW. Disabling camera publishing.");
    publish_images_ = false;
    return;
  }
  publish_images_ = true;
  rendering_thread_ = std::thread(&MujocoCameras::update_loop, this);
}

void MujocoCameras::close()
{
  publish_images_ = false;
  if (rendering_thread_.joinable())
  {
    rendering_thread_.join();
  }
}

void MujocoCameras::update_loop()
{
  // We create an offscreen context specific to this process for managing camera rendering.
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(1, 1, "", NULL, NULL);
  if (window == nullptr)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to create the offscreen GLFW window. Camera images will not be published.");
    publish_images_ = false;
    return;
  }
  glfwMakeContextCurrent(window);

  // Initialization of the context and data structures has to happen in the rendering thread
  RCLCPP_INFO(node_->get_logger(), "Initializing rendering for cameras");
  if (cameras_.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "No cameras were registered; the camera rendering loop cannot publish point clouds.");
  }
  initialize_visual_options();
  configure_depth_rendering_visibility();

  // Initialize data for camera rendering
  mj_camera_data_ = mj_makeData(mj_model_);
  RCLCPP_INFO(node_->get_logger(), "Starting the camera rendering loop, publishing at %f Hz", camera_publish_rate_);

  initialize_scene();
  initialize_renderer_context();

  // TODO: Support per-camera publish rates?
  const mjtNum publish_period = 1.0 / camera_publish_rate_;
  while (rclcpp::ok() && publish_images_)
  {
    update();
    simulation_clock_.sleep(publish_period, publish_images_);
  }

  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
  mj_deleteData(mj_camera_data_);
  glfwDestroyWindow(window);
  RCLCPP_INFO(node_->get_logger(), "Camera rendering loop stopped.");
}

void MujocoCameras::update()
{
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                       "Camera update loop is running for %zu camera(s).", cameras_.size());
  // Rendering is done offscreen
  mjr_setBuffer(mjFB_OFFSCREEN, &mjr_con_);

  // Step 1: Lock the sim and copy data for use in all camera rendering.
  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    mjv_copyData(mj_camera_data_, mj_model_, mj_data_);
  }

  // Step 2: Render the scene and copy depth data to camera containers.
  for (auto& camera : cameras_)
  {
    // Render scene
    mjv_updateScene(mj_model_, mj_camera_data_, &mjv_opt_, NULL, &camera.mjv_cam, mjCAT_ALL, &mjv_scn_);
    mjr_render(camera.viewport, &mjv_scn_, &mjr_con_);

    mjr_readPixels(nullptr, camera.depth_buffer.data(), camera.viewport, &mjr_con_);
  }

  // Step 3: Convert the rendered depth maps to point clouds.
  const float near = static_cast<float>(mj_model_->vis.map.znear * mj_model_->stat.extent);
  const float far = static_cast<float>(mj_model_->vis.map.zfar * mj_model_->stat.extent);
  const float depth_scale = 1.0f - near / far;
  for (auto& camera : cameras_)
  {
    camera.pointcloud_ready = false;
    try
    {
      // MuJoCo stores non-linear depth bottom-to-top; projection expects linear top-to-bottom values.
      linearize_and_flip_depth(camera, near, depth_scale);
      const auto pointcloud = camera.depth_to_pointcloud->convert(camera.depth_buffer_flipped);
      populate_pointcloud_message(camera, pointcloud);
      camera.pointcloud_ready = true;
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "Camera '%s' failed to create a point cloud: %s", camera.name.c_str(), error.what());
    }
  }

  // Step 4: Publish point clouds.
  for (auto& camera : cameras_)
  {
    if (!camera.pointcloud_ready)
    {
      continue;
    }
    const auto time = node_->now();
    camera.pointcloud_message.header.stamp = time;
    camera.pointcloud_pub->publish(camera.pointcloud_message);
    if (!camera.has_published)
    {
      camera.has_published = true;
      RCLCPP_INFO(node_->get_logger(), "Published the first point cloud for camera '%s' on '%s'.", camera.name.c_str(),
                  camera.pointcloud_topic.c_str());
    }
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Publishing '%s': %u points, %zu subscriber(s).", camera.pointcloud_topic.c_str(),
                         camera.pointcloud_message.width * camera.pointcloud_message.height,
                         camera.pointcloud_pub->get_subscription_count());
  }
}

}  // namespace mujoco_ros2_control
