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

#include "mujoco_ros2_control/mujoco_simulation.hpp"
#include "mujoco_ros2_control/physics_loop_synchronizer.hpp"
#include "array_safety.h"

#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>


#include <hardware_interface/version.h>
#include <rclcpp/version.h>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include "lodepng.h"

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

using namespace std::chrono_literals;

// constants
const int kErrorLength = 1024;           // load error string length

using Seconds = std::chrono::duration<double>;

namespace mujoco_ros2_control
{
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

/**
 * No-op UI adapter to support running the drivers in a headless environment.
 */
class HeadlessAdapter : public mj::PlatformUIAdapter
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

/**
 * GlfwAdapter subclass that overrides right-arrow-key handling so that a single
 * simulation step is driven exclusively by the ROS control loop rather than by
 * MuJoCo's built-in viewer.
 *
 * When the simulation is paused, MuJoCo's default GlfwAdapter advances the
 * physics by one step (mj_step) on each right-arrow press or key-repeat event.
 * This class suppresses that native behaviour and instead sets step_requested_,
 * which the ROS control loop polls to advance the simulation.  This ensures
 * that ros2_controller read/update/write cycles are executed for every step and
 * that controller state remains consistent with the physics.
 *
 * All other keys are forwarded to the parent class unchanged so that the rest
 * of the MuJoCo viewer UI (play/pause, reset, rendering options, etc.) works
 * as normal.
 */
class ROS2ControlGlfwAdapter : public mj::GlfwAdapter
{
public:
  explicit ROS2ControlGlfwAdapter(std::atomic<bool>& step_requested) : step_requested_(step_requested)
  {
  }

protected:
  void OnKey(int key, int scancode, int act) override
  {
    // Intercept the right arrow key so only the ROS loop advances the physics,
    // preventing double-stepping (MuJoCo's native handler would also call mj_step).
    if (key == GLFW_KEY_RIGHT)
    {
      if (act == GLFW_PRESS || act == GLFW_REPEAT)
      {
        step_requested_.store(true);
      }
      return;
    }

    // Forward all other keys so normal UI behaviour is preserved.
    mj::GlfwAdapter::OnKey(key, scancode, act);
  }

private:
  std::atomic<bool>& step_requested_;
};

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
static std::string getExecutableDir()
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

// Load all shared-library plugins from a directory. Unlike
// mj_loadAllPluginLibraries(), this follows the symlinks created by
// `colcon build --symlink-install`.
static void loadPluginsFromDirectory(const std::string& plugin_dir)
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

// Scan for MuJoCo extension libraries next to the executable and in packages
// registered in the ament resource index under the `mujoco_plugins` resource.
static void scanPluginLibraries()
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

  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();
  if (!executable_dir.empty())
  {
    loadPluginsFromDirectory(executable_dir + "/" + MUJOCO_PLUGIN_DIR);
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
    loadPluginsFromDirectory(plugin_dir.string());
  }
}

//------------------------------------------- simulation
//-------------------------------------------

static const char* Diverged(int disableflags, const mjData* d)
{
  if (disableflags & mjDSBL_AUTORESET)
  {
    for (mjtWarning w : { mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS })
    {
      if (d->warning[w].number > 0)
      {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

struct LoadedModel
{
  mjModel* model{ nullptr };
  mjSpec* spec{ nullptr };
};

static LoadedModel loadModelFromFile(const char* file, mj::Simulate& sim)
{
  LoadedModel loaded;

  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // load and compile
  char loadError[kErrorLength] = "";
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename) > 4 && !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                                                     mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
  {
    loaded.model = mj_loadModel(filename, nullptr);
    if (!loaded.model)
    {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  }
  else
  {
    loaded.spec = mj_parseXML(filename, nullptr, loadError, kErrorLength);
    if (loaded.spec)
    {
      loaded.model = mj_compile(loaded.spec, nullptr);
      if (!loaded.model)
      {
        mju::strcpy_arr(loadError, mjs_getError(loaded.spec));
      }
    }

    // remove trailing newline character from loadError
    if (loadError[0])
    {
      auto error_length = mju::strlen_arr(loadError);
      if (loadError[error_length - 1] == '\n')
      {
        loadError[error_length - 1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  if (!loaded.model)
  {
    std::printf("%s\n", loadError);
    mju::strcpy_arr(sim.load_error, loadError);
    if (loaded.spec)
    {
      mj_deleteSpec(loaded.spec);
      loaded.spec = nullptr;
    }
    return loaded;
  }

  // compiler warning: print and pause
  if (loadError[0])
  {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  // if no error and load took more than 1/4 seconds, report load time
  else if (load_seconds > 0.25)
  {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);
  return loaded;
}

MujocoSimulation::~MujocoSimulation()
{
  shutdown();

  // Cleanup data and the model, if they haven't been
  if (mj_data_)
  {
    mj_deleteData(mj_data_);
  }
  if (mj_data_control_)
  {
    mj_deleteData(mj_data_control_);
  }
  if (mj_model_)
  {
    mj_deleteModel(mj_model_);
  }
  if (mj_spec_)
  {
    mj_deleteSpec(mj_spec_);
  }
}

bool MujocoSimulation::initialize(rclcpp::Node::SharedPtr node, const std::string& model_path,
                                  double sim_speed_factor, bool headless)
{
  node_ = node;
  model_path_ = model_path;
  sim_speed_factor_ = sim_speed_factor;
  headless_ = headless;

  RCLCPP_INFO(get_logger(), "Running physics without wall-clock synchronization.");

  RCLCPP_INFO_EXPRESSION(get_logger(), headless_, "Running in HEADLESS mode.");

  // We essentially reconstruct the 'simulate.cc::main()' function here, and
  // launch a Simulate object with all necessary rendering process/options
  // attached.

  // scan for libraries in the plugin directory to load additional plugins
  RCLCPP_INFO(get_logger(), "Scanning plugin libraries...");
  scanPluginLibraries();

  // Retain scope
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultPerturb(&pert_);

  // There is a timing issue here as the rendering context must be attached to
  // the executing thread, but we require the simulation to be available on
  // init. So we spawn the sim in the rendering thread prior to proceeding with
  // initialization.
  RCLCPP_INFO(get_logger(), "Initializing simulation...");
  auto sim_ready = std::make_shared<std::promise<void>>();
  std::future<void> sim_ready_future = sim_ready->get_future();

  if (headless_)
  {
    sim_ = std::make_unique<mj::Simulate>(std::make_unique<HeadlessAdapter>(), &cam_, &opt_, &pert_,
                                          /* is_passive = */ false);

    // Notify sim that we are ready
    sim_ready->set_value();
  }
  else
  {
    // Launch the UI loop in the background
    ui_thread_ = std::thread([this, sim_ready]() {
      sim_ = std::make_unique<mj::Simulate>(std::make_unique<ROS2ControlGlfwAdapter>(keyboard_step_requested_), &cam_,
                                            &opt_, &pert_,
                                            /* is_passive = */ false);

      // Add ros2 control icon for the taskbar
      std::string icon_location =
          ament_index_cpp::get_package_share_directory("mujoco_ros2_control") + "/resources/mujoco_logo.png";
      std::vector<unsigned char> image;
      unsigned width, height;
      unsigned error = lodepng::decode(image, width, height, icon_location);

      // Only process the icon if we successfully loaded it. Otherwise, just proceed without
      if (error)
      {
        RCLCPP_WARN(get_logger(), "LodePNG error %u: %s. Icon file not loaded: %s", error, lodepng_error_text(error),
                    icon_location.c_str());
      }
      else
      {
        GLFWimage icon;
        icon.width = width;
        icon.height = height;
        icon.pixels = image.data();
        glfwSetWindowIcon(glfwGetCurrentContext(), 1, &icon);
      }

      // Set glfw window size to max size of the primary monitor
      const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
      glfwSetWindowSize(glfwGetCurrentContext(), mode->width, mode->height);

      // Hide UI panels programmatically
      sim_->ui0_enable = false;  // Hide left panel
      sim_->ui1_enable = false;  // Hide right panel

      // Notify sim that we are ready
      sim_ready->set_value();

      // Blocks until terminated
      RCLCPP_INFO(get_logger(), "Starting the MuJoCo rendering thread...");
      sim_->RenderLoop();
    });
  }

  if (sim_ready_future.wait_for(10s) == std::future_status::timeout)
  {
    RCLCPP_FATAL(get_logger(), "Timed out waiting to start simulation rendering!");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Sim ready, continuing initialization...");

  // We maintain a pointer to the mutex so that we can lock from here, too.
  // Is this a terrible idea? Maybe, but it lets us use their libraries as is...
  sim_mutex_ = &sim_->mtx;

  // Load the model and data prior to hw registration and starting the physics thread
  sim_->LoadMessage(model_path_.c_str());

  // Time publisher will be pushed from the physics_thread_
  RCLCPP_INFO(get_logger(), "Constructing clock publisher.");
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  clock_realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>>(clock_publisher_);

  // Finish initialization by loading the model and initializing the model and control data containers.
  RCLCPP_INFO(get_logger(), "Loading model...");
  const LoadedModel loaded = loadModelFromFile(model_path_.c_str(), *sim_);
  mj_model_ = loaded.model;
  mj_spec_ = loaded.spec;
  if (!mj_model_)
  {
    RCLCPP_FATAL(get_logger(), "MuJoCo failed to load the model");
    return false;
  }

  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    mj_data_ = mj_makeData(mj_model_);
    mj_data_control_ = mj_makeData(mj_model_);
  }
  if (!mj_data_ || !mj_data_control_)
  {
    RCLCPP_FATAL(get_logger(), "Could not allocate mjData for '%s'", model_path_.c_str());
    return false;
  }

  return true;
}

void MujocoSimulation::capture_initial_state()
{
  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
  initial_qpos_.assign(mj_data_->qpos, mj_data_->qpos + mj_model_->nq);
  initial_qvel_.assign(mj_data_->qvel, mj_data_->qvel + mj_model_->nv);
  initial_ctrl_.assign(mj_data_->ctrl, mj_data_->ctrl + mj_model_->nu);
}

void MujocoSimulation::set_reset_callback(ResetCallback callback)
{
  reset_callback_ = std::move(callback);
}

void MujocoSimulation::start_physics_thread(PhysicsLoopSynchronizer* synchronizer)
{
  if (!synchronizer)
  {
    throw std::invalid_argument("A physics-loop synchronizer is required");
  }

  // Disable the rangefinder flag at startup so that we don't get the yellow lines.
  // We can still turn this on manually if desired.
  sim_->opt.flags[mjVIS_RANGEFINDER] = false;
  // Turn off site rendering so that the visualization is more realistic.
  // These can still be turned on in the visualizer.
  for (int i = 0; i < mjNGROUP; i++)
  {
    sim_->opt.sitegroup[i] = 0;
  }

  // When the interface is activated, we start the physics engine.
  physics_thread_ = std::thread([this, synchronizer]() {
    // Load the simulation and do an initial forward pass
    RCLCPP_INFO(get_logger(), "Starting the MuJoCo physics thread...");
    if (this->headless_)
    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
      sim_->m_ = mj_model_;
      sim_->d_ = mj_data_;
      mju::strcpy_arr(sim_->filename, model_path_.c_str());
    }
    else
    {
      sim_->Load(mj_model_, mj_data_, model_path_.c_str());
    }
    // lock the sim mutex
    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
      mj_forward(mj_model_, mj_data_);
    }
    // Blocks until terminated
    physics_loop(*synchronizer);
  });
}

mjtNum MujocoSimulation::simulation_time() const
{
  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
  return mj_data_ ? mj_data_->time : 0.0;
}

bool MujocoSimulation::exit_requested() const
{
  return !sim_ || sim_->exitrequest.load();
}

void MujocoSimulation::shutdown()
{
  // If sim_ is created and running, clean shut it down
  if (sim_)
  {
    sim_->exitrequest.store(true);
    sim_->run = false;

    if (physics_thread_.joinable())
    {
      physics_thread_.join();
    }
    if (ui_thread_.joinable())
    {
      ui_thread_.join();
    }
  }

}

void MujocoSimulation::reset_world_state()
{
  /// @note This method assumes sim_mutex_ is already held by the caller

  // Save the simulation time to preserve ROS clock continuity
  const mjtNum saved_time = mj_data_->time;

  // Reset all positions, velocities and controls to initial state
  std::copy(initial_qpos_.begin(), initial_qpos_.end(), mj_data_->qpos);
  std::copy(initial_qvel_.begin(), initial_qvel_.end(), mj_data_->qvel);
  std::copy(initial_ctrl_.begin(), initial_ctrl_.end(), mj_data_->ctrl);

  // Reset actuator activations (for muscles and similar)
  std::fill(mj_data_->act, mj_data_->act + mj_model_->na, 0.0);

  // Reset warmstart accelerations
  std::fill(mj_data_->qacc_warmstart, mj_data_->qacc_warmstart + mj_model_->nv, 0.0);

  // Reset sensor data
  std::fill(mj_data_->sensordata, mj_data_->sensordata + mj_model_->nsensordata, 0.0);

  // Reset actuator forces
  std::fill(mj_data_->actuator_force, mj_data_->actuator_force + mj_model_->nu, 0.0);

  // Reset applied forces
  std::fill(mj_data_->qfrc_applied, mj_data_->qfrc_applied + mj_model_->nv, 0.0);
  std::fill(mj_data_->xfrc_applied, mj_data_->xfrc_applied + 6 * mj_model_->nbody, 0.0);

  // Restore simulation time to preserve ROS clock continuity
  mj_data_->time = saved_time;

  // Run forward dynamics to update derived quantities
  mj_forward(mj_model_, mj_data_);

  // Copy to control data for reads - this ensures the physics loop uses the reset state
  mj_copyData(mj_data_control_, mj_model_, mj_data_);

  // Delegate HW-side bookkeeping (command/state interface sync, etc.)
  if (reset_callback_)
  {
    reset_callback_();
  }
}

void MujocoSimulation::physics_loop(const PhysicsLoopSynchronizer& synchronizer)
{
  mjtNum previous_sim_time = 0;

  initialize_force_buffers();

  while (!sim_->exitrequest.load())
  {
    // The loop is intentionally unpaced, but yielding prevents it from starving
    // ROS callbacks, controller_manager, and the rendering thread.
    std::this_thread::yield();

    mjtNum validation_time = 0;

    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);

      handle_ui_reset(previous_sim_time);

      if (!mj_model_ || !mj_data_)
      {
        continue;
      }

      const int force_buffer_size = 6 * mj_model_->nbody;

      if (!sim_->run)
      {
        capture_viewer_forces(force_buffer_size);
        run_paused_simulation(force_buffer_size);
        previous_sim_time = mj_data_->time;
        continue;
      }

      // Resuming continuous simulation cancels any unfinished manual-step request.
      if (pending_steps_.load() > 0)
      {
        RCLCPP_WARN(get_logger(), "Simulation resumed while %u step(s) were still pending; aborting.",
                    pending_steps_.load());
        pending_steps_.store(0);
      }

      validation_time = mj_data_->time;
    }  // Release sim_mutex_ before command validation is allowed to block.

    // Do not advance MuJoCo until every command expected at this simulation
    // time is available. sim_mutex_ is not held here, allowing command and
    // controller callbacks to run and satisfy the validation condition.
    synchronizer.sync_physics_loop();

    if (sim_->exitrequest.load())
    {
      break;
    }

    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);

      if (sim_->exitrequest.load())
      {
        break;
      }

      if (handle_ui_reset(previous_sim_time))
      {
        // The state changed while command validation was in progress. Validate
        // commands again against the reset state before advancing physics.
        continue;
      }

      if (!mj_model_ || !mj_data_ || !sim_->run)
      {
        continue;
      }

      // Do not apply commands validated for a state that was reset while waiting.
      if (mj_data_->time != validation_time)
      {
        continue;
      }

      const int force_buffer_size = 6 * mj_model_->nbody;
      capture_viewer_forces(force_buffer_size);
      run_active_simulation_without_realtime(force_buffer_size);
      previous_sim_time = mj_data_->time;
    }
  }
}

// Perform exactly one timestep without wall-clock synchronization.
void MujocoSimulation::run_active_simulation_without_realtime(int force_buffer_size)
{
  copy_control_inputs();
  compose_external_forces(force_buffer_size);

  // Advance by exactly mj_model_->opt.timestep.
  mj_step(mj_model_, mj_data_);

  const char* divergence_message =
      Diverged(mj_model_->opt.disableflags, mj_data_);

  // Commit the resulting state before publishing /clock.
  mj_copyData(mj_data_control_, mj_model_, mj_data_);

  if (divergence_message)
  {
    sim_->run = 0;
    mju::strcpy_arr(sim_->load_error, divergence_message);

    // The step occurred, so publish its resulting simulation time.
    publish_clock();
    return;
  }

  restore_external_forces_for_viewer(force_buffer_size);

  sim_->AddToHistory();
  step_count_.fetch_add(1);
  update_sim_display();

  // Publish only after the controller-facing state is ready.
  publish_clock();
}

void MujocoSimulation::publish_clock()
{
  auto sim_time = mj_data_->time;
  int32_t sim_time_sec = static_cast<int32_t>(std::floor(sim_time));
  uint32_t sim_time_nanosec = static_cast<uint32_t>((sim_time - sim_time_sec) * 1e9);
  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);

  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time_ros;
// fixing for different naming convention on humble vs everything else
#if ROS_DISTRO_HUMBLE
  clock_realtime_publisher_->tryPublish(sim_time_msg);
#else
  clock_realtime_publisher_->try_publish(sim_time_msg);
#endif
}

void MujocoSimulation::update_sim_display()
{
  if (headless_)
  {
    return;
  }

  // Only write user_texts_new_ when the render thread has consumed the previous
  // update (newtextrequest == 0). Use compare_exchange to atomically claim the
  // slot: if it fails, the render thread hasn't swapped yet, so skip this
  // update — the display will be refreshed on the next physics step instead.
  // This avoids a data race: the render thread swaps user_texts_new_ (without
  // holding any mutex) while we clear/populate it.
  int expected = 0;
  if (!sim_->newtextrequest.compare_exchange_strong(expected, 1))
  {
    return;  // render thread hasn't consumed the last update yet, skip
  }

  const std::string status = sim_->run ? "Running" : "Paused";
  sim_->user_texts_new_.clear();
  sim_->user_texts_new_.emplace_back(mjFONT_NORMAL, mjGRID_TOPRIGHT, "Status\nSteps",
                                     status + "\n" + std::to_string(step_count_.load()));
}

}  // namespace mujoco_ros2_control
