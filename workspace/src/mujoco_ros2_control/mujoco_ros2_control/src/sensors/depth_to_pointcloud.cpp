
#include "mujoco_ros2_control/sensors/depth_to_pointcloud.hpp"

#include <cmath>
#include <stdexcept>

namespace mujoco_ros2_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

DepthToPointcloud::DepthToPointcloud(uint32_t width, uint32_t height, double vertical_fov_degrees)
  : width_(width)
  , height_(height)
  , focal_length_(static_cast<float>(height / (2.0 * std::tan(vertical_fov_degrees * kPi / 360.0))))
  , principal_x_(static_cast<float>(width) / 2.0F)
  , principal_y_(static_cast<float>(height) / 2.0F)
{
  if (width == 0 || height == 0 || focal_length_ <= 0.0F || !std::isfinite(focal_length_))
  {
    throw std::invalid_argument("A point cloud projector requires a valid camera resolution and field of view.");
  }
}

Pointcloud DepthToPointcloud::convert(const std::vector<float>& depth_image) const
{
  if (depth_image.size() != static_cast<size_t>(width_) * height_)
  {
    throw std::invalid_argument("Depth image dimensions do not match the point cloud projector.");
  }

  Pointcloud pointcloud;
  pointcloud.reserve(depth_image.size());
  for (uint32_t row = 0; row < height_; ++row)
  {
    for (uint32_t column = 0; column < width_; ++column)
    {
      const auto index = static_cast<size_t>(row) * width_ + column;
      pointcloud.push_back(project_pixel(column, row, depth_image[index]));
    }
  }
  return pointcloud;
}

Point3f DepthToPointcloud::project_pixel(uint32_t column, uint32_t row, float depth) const
{
  return { (static_cast<float>(column) - principal_x_) * depth / focal_length_,
           (static_cast<float>(row) - principal_y_) * depth / focal_length_, depth };
}

}  // namespace mujoco_ros2_control
