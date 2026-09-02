
#include "mujoco_ros2_control/system_interface/mujoco_model_validation.hpp"

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{

bool validate_mujoco_joint_names(const mjModel* model, const rclcpp::Logger& logger)
{
  int num_joints_without_name = 0;
  for (int i = 0; i < model->njnt; ++i)
  {
    const char* joint_name = mj_id2name(model, mjtObj::mjOBJ_JOINT, i);
    const int joint_type = model->jnt_type[i];
    if (!joint_name && joint_type != mjJNT_FREE)
    {
      num_joints_without_name++;
    }
  }
  if (num_joints_without_name)
  {
    RCLCPP_FATAL(logger, "%d joints in the mjcf don't have names. All non-free joints must have names.",
                 num_joints_without_name);
    return false;
  }
  return true;
}

}  // namespace mujoco_ros2_control
