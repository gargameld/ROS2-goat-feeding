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

namespace mujoco_ros2_control
{

/**
 * @brief Register every MuJoCo engine extension available in this installation.
 *
 * These are MuJoCo's own native plugins (shared libraries implementing mjpPlugin, such as the
 * lidar sensor), not the ros2_control plugins loaded by ControlPluginLoader. They must be
 * registered before a model is compiled, because the MJCF resolves its `<extension>` declarations
 * against the registered set.
 *
 * Libraries are discovered in two places: the `mujoco_plugin` directory beside the running
 * executable, and every ROS package that registers a `mujoco_plugins` ament resource.
 */
void load_mujoco_extensions();

}  // namespace mujoco_ros2_control
