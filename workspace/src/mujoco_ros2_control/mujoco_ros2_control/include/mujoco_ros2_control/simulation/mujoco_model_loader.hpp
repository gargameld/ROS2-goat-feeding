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
