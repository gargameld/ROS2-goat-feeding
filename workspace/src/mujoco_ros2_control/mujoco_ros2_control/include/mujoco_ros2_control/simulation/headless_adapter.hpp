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

#include <utility>

#include <mujoco/mujoco.h>

#include "platform_ui_adapter.h"

namespace mujoco_ros2_control
{

/**
 * No-op UI adapter to support running the drivers in a headless environment.
 */
class HeadlessAdapter : public ::mujoco::PlatformUIAdapter
{
public:
  HeadlessAdapter() = default;
  ~HeadlessAdapter() override = default;

  std::pair<double, double> GetCursorPosition() const override
  {
    return { 0.0, 0.0 };
  }
  double GetDisplayPixelsPerInch() const override
  {
    return 96.0;
  }
  std::pair<int, int> GetFramebufferSize() const override
  {
    return { 800, 600 };
  }
  std::pair<int, int> GetWindowSize() const override
  {
    return { 800, 600 };
  }
  bool IsGPUAccelerated() const override
  {
    return false;
  }
  void PollEvents() override
  {
  }
  void SetClipboardString(const char* /*text*/) override
  {
  }
  void SetVSync(bool /*enabled*/) override
  {
  }
  void SetWindowTitle(const char* /*title*/) override
  {
  }
  bool ShouldCloseWindow() const override
  {
    return false;
  }
  void SwapBuffers() override
  {
  }
  void ToggleFullscreen() override
  {
  }

  bool IsLeftMouseButtonPressed() const override
  {
    return false;
  }
  bool IsMiddleMouseButtonPressed() const override
  {
    return false;
  }
  bool IsRightMouseButtonPressed() const override
  {
    return false;
  }

  bool IsAltKeyPressed() const override
  {
    return false;
  }
  bool IsCtrlKeyPressed() const override
  {
    return false;
  }
  bool IsShiftKeyPressed() const override
  {
    return false;
  }

  bool IsMouseButtonDownEvent(int /*act*/) const override
  {
    return false;
  }
  bool IsKeyDownEvent(int /*act*/) const override
  {
    return false;
  }

  int TranslateKeyCode(int /*key*/) const override
  {
    return 0;
  }
  mjtButton TranslateMouseButton(int /*button*/) const override
  {
    return mjBUTTON_NONE;
  }

  bool RefreshMjrContext(const mjModel* /*m*/, int /*fontscale*/) override
  {
    return false;
  }
};

//------------------------------------------- simulation

}  // namespace mujoco_ros2_control
