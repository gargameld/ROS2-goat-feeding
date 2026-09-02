
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include <rclcpp/rclcpp.hpp>

#include "simulate.h"  // must be on your include path, handled by CMake

#include "mujoco_ros2_control/simulation/mujoco_simulation_clock.hpp"

namespace mujoco_ros2_control
{

class PhysicsLoopSynchronizer;

/**
 * @brief ROS 2-based container for the mujoco Simulate application.
 *
 * This class wraps the MuJoCo simulation and Simulate application, while providing necessary
 * hooks to the ros2_control system interface to enable interaction with the sim using
 * "normal" ROS 2 constructs.
 *
 * The simulation always runs headless: there is no window, no render loop and no viewer
 * interaction. This class owns the mujoco model and data along with the physics thread, and
 * publishes the simulated time to /clock.
 *
 * Thread safety is still somewhat messy, as callers are provided with a simulation mutex that
 * locks the model and data while the actual mujoco engine moves the sim forward. Callers
 * need to be wary of locking that mutex external to this class, as it can have significant
 * consequences on the simulation's speed.
 *
 */
class MujocoSimulation
{
public:
  /**
   * @brief Construct a new Mujoco Simulation object. This is a no-op until initialization.
   */
  MujocoSimulation() = default;

  ~MujocoSimulation();

  /**
   * @brief Construct the headless Simulate application.
   *
   * This initializes the Simulate app with a no-op UI adapter and sets up the required
   * publishers using the provided node.
   */
  bool initialize(rclcpp::Node::SharedPtr node, const std::string& model_path);

  /**
   * @brief Start the physics thread. Must be called after load_model().
   */
  void start_physics_thread(PhysicsLoopSynchronizer* synchronizer);

  /**
   * @brief Stop the physics thread if it is running.
   */
  void shutdown();

  /**
   * @brief Accessor for the mujoco model.
   */
  mjModel* model()
  {
    return mj_model_;
  }

  /**
   * @brief Accessor for the editable specification used to compile the model.
   */
  mjSpec* spec()
  {
    return mj_spec_;
  }

  /**
   * @brief Accessor for the mujoco data.
   */
  mjData* data()
  {
    return mj_data_;
  }

  /**
   * @brief Accessor for the mujoco control data.
   */
  mjData* control_data()
  {
    return mj_data_control_;
  }

  /**
   * @brief Accessor for the mutex which locks access to the data and model.
   */
  std::recursive_mutex& mutex() const
  {
    RCLCPP_WARN_EXPRESSION(logger_, sim_mutex_ == nullptr, "Sim recursive mutex is still nullptr");
    return *sim_mutex_;
  }

  /**
   * @brief Accessor for the stacking applied forces, these values will be added to `xfrc_applied`.
   *
   * TODO: Remove this and provide consisted access for write-able mujoco data.
   */
  std::vector<mjtNum>& xfrc_plugin_desired()
  {
    return xfrc_plugin_desired_;
  }

  /**
   * @brief Publish the current simulation state to the controller-facing data.
   *
   * The physics loop does this after every step. It is only needed from the outside after the
   * simulation state has been seeded directly, so the first read() observes it.
   */
  void sync_control_data();

  /**
   * @brief Return a thread-safe snapshot of the current simulation time.
   */
  mjtNum simulation_time() const;

  /**
   * @brief Accessor for the clock shared by every consumer of simulation time.
   */
  const MujocoSimulationClock& clock() const
  {
    return simulation_clock_;
  }

  /**
   * @brief Return whether shutdown has been requested for the simulation.
   */
  bool exit_requested() const;

private:
  /**
   * @brief Loops the physics simulation until asked to terminate.
   */
  void physics_loop(const PhysicsLoopSynchronizer& synchronizer);

  /**
   * @brief Advance the running simulation by exactly one MuJoCo timestep.
   */
  void run_active_simulation_without_realtime(int force_buffer_size);

  /**
   * @brief Keep the controller-facing state current while the simulation is paused.
   */
  void run_paused_simulation(int force_buffer_size);

  void initialize_force_buffers();
  void copy_control_inputs();
  void apply_plugin_forces(int force_buffer_size);

  rclcpp::Logger get_logger() const
  {
    return logger_;
  }

  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("MujocoSimulation");

  // ROS node (owned by the HW interface, used here for services and clock publisher).
  rclcpp::Node::SharedPtr node_;

  // System information
  std::string model_path_;

  // MuJoCo data pointers
  mjSpec* mj_spec_{ nullptr };
  mjModel* mj_model_{ nullptr };
  mjData* mj_data_{ nullptr };

  // Data container for control data
  mjData* mj_data_control_{ nullptr };

  // TODO: Conslidate the control and this buffer to provide consistent, clear access for
  //       for mujoco data.
  //
  // Dedicated buffer for plugin xfrc contributions.
  //
  // mj_copyData overwrites mj_data_control_->xfrc_applied every step, so we cannot rely on it
  // to hold plugin forces across physics iterations. Instead, the control thread (read())
  // zeroes mj_data_control_->xfrc_applied before every plugin update, lets plugins write fresh
  // forces, then copies the result here. The physics thread applies this buffer before each
  // mj_step. Because it is never touched by mj_copyData, it holds the last plugin contribution
  // cleanly until the next control cycle.
  std::vector<mjtNum> xfrc_plugin_desired_;  ///< plugin forces, set once per control cycle

  // Required by the Simulate constructor; unused while running headless.
  mjvCamera cam_;
  mjvOption opt_;
  mjvPerturb pert_;

  // Primary simulate object
  std::unique_ptr<mujoco::Simulate> sim_;

  // Thread running the physics loop
  std::thread physics_thread_;

  // Publishes simulation time to /clock, once per physics step
  SimulationClockPublisher clock_publisher_;

  // Mutex used inside simulate.h for protecting model/data, we keep a reference
  // here to protect access to shared data.
  // TODO: It would be far better to put all relevant data into a single container with accessors
  //       in a common location rather than passing around the raw pointer to the mutex, but it would
  //       require more work to pull it out of simulate.h.
  std::recursive_mutex* sim_mutex_{ nullptr };

  // Simulation-time reads, waits, and the camera update time shared between the rendering
  // thread and the physics loop.
  MujocoSimulationClock simulation_clock_{ *this };

};

}  // namespace mujoco_ros2_control
