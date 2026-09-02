

#include "mujoco_ros2_control/simulation/mujoco_simulation.hpp"
#include "mujoco_ros2_control/simulation/physics_loop_synchronizer.hpp"

#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{
namespace
{

/// Report why the last step diverged, or nullptr when it did not.
const char* divergence_reason(int disableflags, const mjData* d)
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

}  // namespace

void MujocoSimulation::initialize_force_buffers()
{
  xfrc_plugin_desired_.assign(6 * mj_model_->nbody, 0.0);
}

void MujocoSimulation::copy_control_inputs()
{
  // ctrl has one value per actuator.
  mju_copy(
      mj_data_->ctrl,
      mj_data_control_->ctrl,
      static_cast<int>(mj_model_->nu));

  // qfrc_applied has one value per degree of freedom.
  mju_copy(
      mj_data_->qfrc_applied,
      mj_data_control_->qfrc_applied,
      static_cast<int>(mj_model_->nv));
}

void MujocoSimulation::apply_plugin_forces(int force_buffer_size)
{
  // Running headless, plugins are the only source of external body forces.
  mju_copy(
      mj_data_->xfrc_applied,
      xfrc_plugin_desired_.data(),
      force_buffer_size);
}

void MujocoSimulation::run_paused_simulation(int force_buffer_size)
{
  // The simulation only pauses when a step diverged. Keep the controller-facing
  // state current without advancing time.
  mj_copyData(mj_data_control_, mj_model_, mj_data_);

  apply_plugin_forces(force_buffer_size);

  // Update derived quantities without advancing time.
  mj_forward(mj_model_, mj_data_);
}

void MujocoSimulation::physics_loop(const PhysicsLoopSynchronizer& synchronizer)
{
  initialize_force_buffers();

  while (!sim_->exitrequest.load())
  {
    // The loop is intentionally unpaced, but yielding prevents it from starving
    // ROS callbacks, controller_manager, and the rendering thread.
    std::this_thread::yield();

    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);

      if (!mj_model_ || !mj_data_)
      {
        continue;
      }

      if (!sim_->run)
      {
        run_paused_simulation(6 * mj_model_->nbody);
        continue;
      }
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

      if (!mj_model_ || !mj_data_ || !sim_->run)
      {
        continue;
      }

      run_active_simulation_without_realtime(6 * mj_model_->nbody);
    }
  }
}

// Perform exactly one timestep without wall-clock synchronization.
void MujocoSimulation::run_active_simulation_without_realtime(int force_buffer_size)
{
  copy_control_inputs();
  apply_plugin_forces(force_buffer_size);

  // Advance by exactly mj_model_->opt.timestep.
  mj_step(mj_model_, mj_data_);

  const char* divergence_message =
      divergence_reason(mj_model_->opt.disableflags, mj_data_);

  // Commit the resulting state before publishing /clock.
  mj_copyData(mj_data_control_, mj_model_, mj_data_);

  if (divergence_message)
  {
    // Pause the loop: the state is no longer physically meaningful.
    sim_->run = 0;
    RCLCPP_ERROR(get_logger(), "MuJoCo simulation diverged and has been paused: %s", divergence_message);
  }

  // The step occurred either way, so publish its resulting simulation time.
  // Publish only after the controller-facing state is ready.
  clock_publisher_.publish(mj_data_->time);
}


}  // namespace mujoco_ros2_control
