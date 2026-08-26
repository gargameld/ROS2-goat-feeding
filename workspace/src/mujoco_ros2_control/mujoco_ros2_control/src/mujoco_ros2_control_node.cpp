// Copyright 2020 ROS2-Control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/executors.hpp>
#include <realtime_tools/realtime_helpers.hpp>

namespace
{
using ControllerManager = controller_manager::ControllerManager;
using ControllerManagerPtr = std::shared_ptr<ControllerManager>;

// Midpoint real-time priority leaves users room for higher- and lower-priority threads.
constexpr int kSchedPriority = 50;

// The MuJoCo physics loop waits for controller writes before advancing /clock.
// Never wait indefinitely on that same clock here: doing so creates a cycle in
// which physics waits for write() while the control loop waits for physics.
constexpr auto kSimClockPollInterval = std::chrono::microseconds(100);

rclcpp::NodeOptions get_controller_manager_options(int argc, char ** argv)
{
  auto options = controller_manager::get_cm_node_options();
  auto arguments = options.arguments();

  for (int i = 1; i < argc; ++i)
  {
    if (arguments.empty() && std::string(argv[i]) != "--ros-args")
    {
      // Reject arguments before the ROS arguments delimiter.
      continue;
    }
    arguments.push_back(argv[i]);
  }

  options.arguments(arguments);
  return options;
}

rclcpp::Time get_controller_manager_time(const ControllerManagerPtr & controller_manager)
{
  return controller_manager->get_trigger_clock()->now();
}

void lock_memory_if_requested(const ControllerManagerPtr & controller_manager)
{
  const bool has_realtime = realtime_tools::has_realtime_kernel();
  const bool lock_memory =
    controller_manager->get_parameter_or<bool>("lock_memory", has_realtime);

  if (!lock_memory)
  {
    return;
  }

  const auto lock_result = realtime_tools::lock_memory();
  if (!lock_result.first)
  {
    RCLCPP_WARN(
      controller_manager->get_logger(), "Unable to lock the memory: '%s'",
      lock_result.second.c_str());
  }
}

void set_cpu_affinity(const ControllerManagerPtr & controller_manager)
{
  rclcpp::Parameter cpu_affinity_parameter;
  if (!controller_manager->get_parameter("cpu_affinity", cpu_affinity_parameter))
  {
    return;
  }

  std::vector<int> cpus;
  if (cpu_affinity_parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    cpus = {static_cast<int>(cpu_affinity_parameter.as_int())};
  }
  else if (
    cpu_affinity_parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
  {
    const auto cpu_array = cpu_affinity_parameter.as_integer_array();
    std::for_each(
      cpu_array.begin(), cpu_array.end(),
      [&cpus](int cpu) { cpus.push_back(static_cast<int>(cpu)); });
  }

  const auto affinity_result = realtime_tools::set_current_thread_affinity(cpus);
  if (!affinity_result.first)
  {
    RCLCPP_WARN(
      controller_manager->get_logger(), "Unable to set the CPU affinity : '%s'",
      affinity_result.second.c_str());
  }
}

void set_realtime_priority(
  const ControllerManagerPtr & controller_manager, const int thread_priority)
{
  if (!realtime_tools::configure_sched_fifo(thread_priority))
  {
    RCLCPP_WARN(
      controller_manager->get_logger(),
      "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
      "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
      "for details on how to enable realtime scheduling.",
      errno, strerror(errno));
    return;
  }

  RCLCPP_INFO(
    controller_manager->get_logger(),
    "Successful set up FIFO RT scheduling policy with priority %i.", thread_priority);
}

void run_control_loop(
  const ControllerManagerPtr & controller_manager, const int thread_priority,
  const bool use_sim_time, const bool manage_overruns)
{
  set_cpu_affinity(controller_manager);
  set_realtime_priority(controller_manager, thread_priority);

  // MuJoCo-specific change from upstream: the hardware interface must be constructed and running
  // before it can publish the clock. Do not sleep on simulation time before the first write: the
  // physics synchronizer needs that write before it can advance /clock to another deadline.
  // TODO: Revisit this when https://github.com/ros-controls/ros2_control/pull/2654 is resolved.
  controller_manager->get_clock()->wait_until_started();

  const auto period =
    std::chrono::nanoseconds(1'000'000'000 / controller_manager->get_update_rate());

  rclcpp::Time previous_time = get_controller_manager_time(controller_manager);

  auto next_iteration_time = std::chrono::steady_clock::now();

  while (rclcpp::ok())
  {
    const auto current_time = get_controller_manager_time(controller_manager);
    const auto measured_period = current_time - previous_time;
    previous_time = current_time;

    controller_manager->read(get_controller_manager_time(controller_manager), measured_period);
    controller_manager->update(get_controller_manager_time(controller_manager), measured_period);
    controller_manager->write(get_controller_manager_time(controller_manager), measured_period);

    if (use_sim_time)
    {
      const auto simulation_deadline = current_time + period;
      const auto wall_deadline = std::chrono::steady_clock::now() + period;

      // Normally /clock reaches simulation_deadline first. The steady-clock
      // deadline is a watchdog that guarantees another control-loop cycle when
      // the physics synchronizer is waiting for a fresh command.
      while (
        rclcpp::ok() && get_controller_manager_time(controller_manager) < simulation_deadline &&
        std::chrono::steady_clock::now() < wall_deadline)
      {
        std::this_thread::sleep_for(kSimClockPollInterval);
      }
      continue;
    }

    next_iteration_time += period;
    const auto time_now = std::chrono::steady_clock::now();

    if (manage_overruns && next_iteration_time < time_now)
    {
      const double time_diff =
        static_cast<double>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - next_iteration_time)
            .count()) /
        1.e6;
      const double controller_period =
        1.e3 / static_cast<double>(controller_manager->get_update_rate());
      const int overrun_count = static_cast<int>(std::ceil(time_diff / controller_period));

      RCLCPP_WARN_THROTTLE(
        controller_manager->get_logger(), *controller_manager->get_clock(), 1000,
        "Overrun detected! The controller manager missed its desired rate of %d Hz. The loop "
        "took %f ms (missed cycles : %d).",
        controller_manager->get_update_rate(), time_diff + controller_period, overrun_count + 1);

      next_iteration_time += overrun_count * period;
    }

    std::this_thread::sleep_until(next_iteration_time);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  const std::string manager_node_name = "controller_manager";
  auto controller_manager = std::make_shared<ControllerManager>(
    executor, manager_node_name, "", get_controller_manager_options(argc, argv));

  const bool use_sim_time = controller_manager->get_parameter_or("use_sim_time", false);
  lock_memory_if_requested(controller_manager);

  RCLCPP_INFO(
    controller_manager->get_logger(), "update rate is %d Hz",
    controller_manager->get_update_rate());

  const bool manage_overruns =
    controller_manager->get_parameter_or<bool>("overruns.manage", true);
  RCLCPP_INFO(
    controller_manager->get_logger(), "Overruns handling is : %s",
    manage_overruns ? "enabled" : "disabled");

  const int thread_priority =
    controller_manager->get_parameter_or<int>("thread_priority", kSchedPriority);
  RCLCPP_INFO(
    controller_manager->get_logger(), "Spawning %s RT thread with scheduler priority: %d",
    controller_manager->get_name(), thread_priority);

  std::thread control_thread(
    run_control_loop, controller_manager, thread_priority, use_sim_time, manage_overruns);

  executor->add_node(controller_manager);
  executor->spin();
  control_thread.join();
  rclcpp::shutdown();
  return 0;
}
