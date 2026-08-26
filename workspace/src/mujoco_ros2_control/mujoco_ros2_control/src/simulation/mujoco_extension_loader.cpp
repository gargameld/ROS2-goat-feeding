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

#include "mujoco_ros2_control/simulation/mujoco_extension_loader.hpp"

#include <unistd.h>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <new>
#include <string>

#include <mujoco/mujoco.h>

#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>

namespace mujoco_ros2_control
{
namespace
{

// Directory name, relative to the executable, that MuJoCo scans for extension libraries.
constexpr const char* kMujocoExtensionDir = "mujoco_plugin";

// Path of the directory containing the running executable, used to locate the
// extension libraries that sit beside it.
std::string get_executable_directory()
{
  constexpr char kPathSep = '/';
  const char* path = "/proc/self/exe";

  std::string real_path = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success)
    {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      auto written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size)
      {
        realpath.get()[written] = '\0';
        success = true;
      }
      else if (written == -1)
      {
        if (errno == EINVAL)
        {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      }
      else
      {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();

  if (real_path.empty())
  {
    return "";
  }

  for (std::size_t i = real_path.size() - 1; i > 0; --i)
  {
    if (real_path.c_str()[i] == kPathSep)
    {
      return real_path.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

// Load every shared-library extension in a directory. Unlike
// mj_loadAllPluginLibraries(), this follows the symlinks created by
// `colcon build --symlink-install`.
void load_extensions_from_directory(const std::string& plugin_dir)
{
  std::error_code ec;
  if (!std::filesystem::is_directory(plugin_dir, ec))
  {
    return;
  }

  for (const auto& entry : std::filesystem::directory_iterator(plugin_dir, ec))
  {
    if (!entry.is_regular_file(ec))
    {
      continue;
    }

    const auto& path = entry.path();
    if (path.extension() != ".so")
    {
      continue;
    }

    const int first = mjp_pluginCount();
    mj_loadPluginLibrary(path.c_str());
    const int count = mjp_pluginCount() - first;
    if (count <= 0)
    {
      continue;
    }

    std::printf("Plugins registered by library '%s':\n", path.filename().c_str());
    for (int i = first; i < first + count; ++i)
    {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }
}

}  // namespace

void load_mujoco_extensions()
{
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin)
  {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i)
    {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // Look for extension libraries in the `mujoco_plugin` directory beside this executable.
  const std::string executable_dir = get_executable_directory();
  if (!executable_dir.empty())
  {
    load_extensions_from_directory(executable_dir + "/" + kMujocoExtensionDir);
  }

  // Discover extension providers installed anywhere in the sourced ROS
  // environment. The resource content is a path relative to the provider's
  // install prefix (for example lib/mujoco_3d_lidar/mujoco_plugin).
  const auto plugin_packages = ament_index_cpp::get_resources("mujoco_plugins");
  for (const auto& [package_name, unused_prefix] : plugin_packages)
  {
    (void)unused_prefix;
    std::string relative_plugin_dir;
    std::string package_prefix;
    if (!ament_index_cpp::get_resource("mujoco_plugins", package_name, relative_plugin_dir, &package_prefix))
    {
      continue;
    }

    const auto plugin_dir = (std::filesystem::path(package_prefix) / relative_plugin_dir).lexically_normal();
    std::printf("Loading MuJoCo plugins from package '%s': %s\n", package_name.c_str(), plugin_dir.c_str());
    load_extensions_from_directory(plugin_dir.string());
  }
}

}  // namespace mujoco_ros2_control
