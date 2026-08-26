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

#include "mujoco_ros2_control/simulation/mujoco_model_loader.hpp"

#include <chrono>
#include <cstring>

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{
namespace
{

// Longest error message MuJoCo will write back to us.
constexpr int kErrorLength = 1024;

bool has_mjb_extension(const std::string& model_path)
{
  constexpr const char* kBinaryExtension = ".mjb";
  return model_path.size() > 4 && model_path.compare(model_path.size() - 4, 4, kBinaryExtension) == 0;
}

/// Strip the trailing newline MuJoCo leaves on some error strings.
void trim_trailing_newline(char* message)
{
  const std::size_t length = std::strlen(message);
  if (length > 0 && message[length - 1] == '\n')
  {
    message[length - 1] = '\0';
  }
}

}  // namespace

LoadedModel load_model_from_file(const std::string& model_path, const rclcpp::Logger& logger)
{
  LoadedModel loaded;
  char load_error[kErrorLength] = "";

  const auto load_start = std::chrono::steady_clock::now();
  if (has_mjb_extension(model_path))
  {
    loaded.model = mj_loadModel(model_path.c_str(), nullptr);
    if (!loaded.model)
    {
      std::strncpy(load_error, "could not load binary model", sizeof(load_error) - 1);
    }
  }
  else
  {
    loaded.spec = mj_parseXML(model_path.c_str(), nullptr, load_error, kErrorLength);
    if (loaded.spec)
    {
      loaded.model = mj_compile(loaded.spec, nullptr);
      if (!loaded.model)
      {
        std::strncpy(load_error, mjs_getError(loaded.spec), sizeof(load_error) - 1);
        load_error[sizeof(load_error) - 1] = '\0';
      }
    }
    trim_trailing_newline(load_error);
  }
  const auto load_duration = std::chrono::steady_clock::now() - load_start;

  if (!loaded.model)
  {
    RCLCPP_FATAL(logger, "Could not load MuJoCo model '%s': %s", model_path.c_str(), load_error);
    if (loaded.spec)
    {
      mj_deleteSpec(loaded.spec);
      loaded.spec = nullptr;
    }
    return loaded;
  }

  if (load_error[0])
  {
    // The model is usable, but MuJoCo is unhappy with it. Report it and let the caller decide
    // whether to start paused.
    RCLCPP_WARN(logger, "Model compiled with a warning: %s", load_error);
    loaded.compiled_with_warning = true;
  }

  const double load_seconds = std::chrono::duration<double>(load_duration).count();
  RCLCPP_INFO_EXPRESSION(logger, load_seconds > 0.25, "Model loaded in %.2g seconds", load_seconds);
  return loaded;
}

}  // namespace mujoco_ros2_control
