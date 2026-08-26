/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mujoco_ros2_control
{

struct Point3f
{
  float x;
  float y;
  float z;
};

using Pointcloud = std::vector<Point3f>;

/**
 * @brief Projects an organized depth map into camera-frame XYZ points.
 *
 * This class has no ROS or MuJoCo dependencies. It owns the camera intrinsics
 * derived from the image dimensions and vertical field of view.
 */
class DepthToPointcloud
{
public:
  DepthToPointcloud(uint32_t width, uint32_t height, double vertical_fov_degrees);

  /** @brief Converts one row-major depth image, in metres, into an organized point cloud. */
  Pointcloud convert(const std::vector<float>& depth_image) const;

private:
  Point3f project_pixel(uint32_t column, uint32_t row, float depth) const;

  uint32_t width_;
  uint32_t height_;
  float focal_length_;
  float principal_x_;
  float principal_y_;
};

}  // namespace mujoco_ros2_control
