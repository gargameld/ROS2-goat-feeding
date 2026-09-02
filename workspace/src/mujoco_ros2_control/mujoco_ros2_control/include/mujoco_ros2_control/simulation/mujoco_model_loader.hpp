
#pragma once

#include <string>

#include <mujoco/mujoco.h>
#include <rclcpp/logger.hpp>

namespace mujoco_ros2_control
{

/// Outcome of compiling an MJCF (or loading a binary MJB).
struct LoadedModel
{
  /// Compiled model, or nullptr when loading failed.
  mjModel* model{ nullptr };
  /// Editable specification behind the model. Null for binary .mjb models.
  mjSpec* spec{ nullptr };
  /// True when MuJoCo compiled the model but reported a warning about it.
  bool compiled_with_warning{ false };
};

/**
 * @brief Load and compile a MuJoCo model from an MJCF `.xml` or a binary `.mjb` file.
 *
 * Errors and warnings are logged. On failure the returned model is nullptr and any partially
 * created specification has already been freed.
 *
 * @note MuJoCo engine extensions must be registered first (see load_mujoco_extensions()),
 *       because compilation resolves the MJCF's `<extension>` declarations.
 */
LoadedModel load_model_from_file(const std::string& model_path, const rclcpp::Logger& logger);

}  // namespace mujoco_ros2_control
