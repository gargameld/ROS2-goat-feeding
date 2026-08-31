#include "mujoco_ros2_control_plugins/food_control/food_management.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>
#include <utility>

namespace mujoco_ros2_control_plugins
{

FoodManagement::FoodManagement(mjModel* model, mjData* data, double throw_height,
                               std::vector<ParkingFrame> parking_frames)
  : model_(model)
  , data_(data)
  , throw_height_(throw_height)
  , parking_frames_(std::move(parking_frames))
{
}

bool FoodManagement::is_available() const
{
  return model_ != nullptr && data_ != nullptr && !parking_frames_.empty();
}

int FoodManagement::find_free_joint(const std::string& body_name, std::string& error) const
{
  const int body_id = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
  if (body_id < 0)
  {
    error = "No body named '" + body_name + "' exists in the simulation.";
    return -1;
  }

  const int first_joint = model_->body_jntadr[body_id];
  const int joint_count = model_->body_jntnum[body_id];
  for (int joint = first_joint; joint < first_joint + joint_count; ++joint)
  {
    if (model_->jnt_type[joint] == mjJNT_FREE)
    {
      return joint;
    }
  }

  error = "Body '" + body_name + "' has no free joint and cannot be thrown.";
  return -1;
}

bool FoodManagement::throw_food(int parking_index, const std::string& food_name, double x, double y,
                                const double quat[4], std::string& error)
{
  if (!is_available())
  {
    error = "The MuJoCo simulation is unavailable.";
    return false;
  }
  if (parking_index < 1 || static_cast<std::size_t>(parking_index) > parking_frames_.size())
  {
    error = "parking_index must be between 1 and " + std::to_string(parking_frames_.size()) + ".";
    return false;
  }

  const double planar[] = { x, y };
  if (!std::all_of(std::begin(planar), std::end(planar), [](double value) { return std::isfinite(value); }))
  {
    error = "The requested x/y position must be finite.";
    return false;
  }

  const ParkingFrame& parking_frame = parking_frames_[static_cast<std::size_t>(parking_index - 1)];
  if (x < parking_frame.min_x || x > parking_frame.max_x)
  {
    std::ostringstream message;
    message << "Requested x=" << x << " is outside the allowed range [" << parking_frame.min_x << ", "
            << parking_frame.max_x << "] for parking " << parking_index << ".";
    error = message.str();
    return false;
  }
  if (y < parking_frame.min_y || y > parking_frame.max_y)
  {
    std::ostringstream message;
    message << "Requested y=" << y << " is outside the allowed range [" << parking_frame.min_y << ", "
            << parking_frame.max_y << "] for parking " << parking_index << ".";
    error = message.str();
    return false;
  }
  if (!std::all_of(quat, quat + 4, [](double value) { return std::isfinite(value); }))
  {
    error = "The orientation quaternion must contain finite values.";
    return false;
  }

  // Normalise the requested orientation so a lazily specified quaternion is valid.
  double request_quat[4] = { quat[0], quat[1], quat[2], quat[3] };
  const double quat_norm = std::sqrt(request_quat[0] * request_quat[0] + request_quat[1] * request_quat[1] +
                                     request_quat[2] * request_quat[2] + request_quat[3] * request_quat[3]);
  if (quat_norm < 1e-9)
  {
    error = "The orientation quaternion must have a non-zero magnitude.";
    return false;
  }
  for (double& component : request_quat)
  {
    component /= quat_norm;
  }

  const int joint = find_free_joint(food_name, error);
  if (joint < 0)
  {
    return false;
  }

  const int qpos_adr = model_->jnt_qposadr[joint];
  const int dof_adr = model_->jnt_dofadr[joint];

  // Parking frames are map-aligned, so only their configured translation is
  // needed to express the request in the simulation's map/world frame.
  data_->qpos[qpos_adr + 0] = parking_frame.offset_x + x;
  data_->qpos[qpos_adr + 1] = parking_frame.offset_y + y;
  data_->qpos[qpos_adr + 2] = throw_height_;  // constant world-frame drop height
  data_->qpos[qpos_adr + 3] = request_quat[0];
  data_->qpos[qpos_adr + 4] = request_quat[1];
  data_->qpos[qpos_adr + 5] = request_quat[2];
  data_->qpos[qpos_adr + 6] = request_quat[3];

  // Clear the free joint's linear and angular velocity so the item starts at rest.
  for (int dof = 0; dof < 6; ++dof)
  {
    data_->qvel[dof_adr + dof] = 0.0;
  }

  mj_forward(model_, data_);
  return true;
}

}  // namespace mujoco_ros2_control_plugins
