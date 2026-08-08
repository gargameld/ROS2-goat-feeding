# `MujocoSystemInterface` helper extraction report

## Scope and constraints

This refactor changes only the `mujoco_ros2_control` package. The sibling
`mujoco_ros2_control_msgs` and `mujoco_ros2_control_plugins` packages are unchanged.

The refactor is an abstraction-only change:

- The public `MujocoSystemInterface` API is unchanged.
- The order of initialization, validation, logging, state transfer, controller updates, and plugin updates is unchanged.
- Existing ROS 2 Humble/Jazzy conditional behavior is retained.
- Existing actuator, transmission, mimic-joint, sensor, PID, reset, and plugin behavior is retained.
- No synchronization, validation, error-handling, control-mode, or simulation behavior was added or corrected.

The helper API is intentionally placed in the `mujoco_ros2_control::detail` namespace and grouped by responsibility.

## Helper categories

| Category | Header | Implementation | Responsibility |
|---|---|---|---|
| Configuration | `include/mujoco_ros2_control/detail/configuration_helpers.hpp` | `src/detail/configuration_helpers.cpp` | Hardware parameters, node options, model validation, and free-joint discovery |
| Control modes | `include/mujoco_ros2_control/detail/control_mode_helpers.hpp` | `src/detail/control_mode_helpers.cpp` | Enabling and disabling position, velocity, and effort command modes |
| Initial state | `include/mujoco_ros2_control/detail/initial_state_helpers.hpp` | `src/detail/initial_state_helpers.cpp` | Initial-state file parsing, initial pose application, and reset bookkeeping |
| ROS interfaces | `include/mujoco_ros2_control/detail/interface_helpers.hpp` | `src/detail/interface_helpers.cpp` | Construction of exported joint, force-torque, and IMU interfaces |
| Model mapping | `include/mujoco_ros2_control/detail/model_mapping_helpers.hpp` | `src/detail/model_mapping_helpers.cpp` | MuJoCo actuator classification and joint/actuator/interface mapping |
| Plugin discovery | `include/mujoco_ros2_control/detail/plugin_helpers.hpp` | `src/detail/plugin_helpers.cpp` | Discovery of configured plugin namespaces |
| Hardware registration | `include/mujoco_ros2_control/detail/registration_helpers.hpp` | `src/detail/registration_helpers.cpp` | Actuator, joint, transmission, and sensor registration operations |
| Runtime state transfer | `include/mujoco_ros2_control/detail/state_helpers.hpp` | `src/detail/state_helpers.cpp` | Reading MuJoCo state, populating messages, and applying mimic commands |

## Simplified functions

### `MujocoSystemInterface::on_init`

| Helper | Category |
|---|---|
| `load_simulation_configuration` | Configuration |
| `make_mujoco_node_options` | Configuration |
| `validate_mujoco_joint_names` | Configuration |
| `get_hardware_parameter_or` | Configuration |
| `find_free_joint` | Configuration |

The method remains the top-level initialization sequence, while parameter parsing, file validation, node-option construction, model validation, and free-joint lookup are named operations.

### `MujocoSystemInterface::export_state_interfaces`

| Helper | Category |
|---|---|
| `append_joint_state_interfaces` | ROS interfaces |
| `append_force_torque_state_interfaces` | ROS interfaces |
| `append_imu_state_interfaces` | ROS interfaces |

### `MujocoSystemInterface::export_command_interfaces`

| Helper | Category |
|---|---|
| `append_joint_command_interfaces` | ROS interfaces |

### `MujocoSystemInterface::perform_command_mode_switch`

| Helper | Category |
|---|---|
| `update_joint_control_mode` | Control modes |
| `get_joint_actuator_name` | Model mapping, used by `update_joint_control_mode` |

The public method now only preserves the required stop-before-start iteration order.

### `MujocoSystemInterface::read`

| Helper | Category |
|---|---|
| `read_actuator_states` | Runtime state transfer |
| `read_imu_states` | Runtime state transfer |
| `read_force_torque_states` | Runtime state transfer |
| `populate_floating_base_odometry` | Runtime state transfer |

Publishing, transmission propagation, plugin updates, and external-force copying remain in the original method and in their original order.

### `MujocoSystemInterface::write`

| Helper | Category |
|---|---|
| `update_mimic_joint_commands` | Runtime state transfer |

Transmission propagation, PID calculation, and writes to MuJoCo's `ctrl` array remain in the original method.

### `MujocoSystemInterface::register_mujoco_actuators`

| Helper | Category |
|---|---|
| `populate_actuator_model_data` | Hardware registration |
| `initialize_actuator_control` | Hardware registration |
| `append_passive_actuators` | Hardware registration |
| `initialize_actuator_states` | Hardware registration |
| `get_actuator_type` | Model mapping, used by `populate_actuator_model_data` |
| `is_mimic_joint` | Model mapping, used by `append_passive_actuators` |

### `MujocoSystemInterface::register_urdf_joints`

| Helper | Category |
|---|---|
| `get_joint_actuator_name` | Model mapping |
| `configure_mimic_joint` | Hardware registration |
| `find_controllable_actuator` | Hardware registration |
| `initialize_joint_interfaces` | Hardware registration |
| `get_ordered_command_interfaces` | Hardware registration |
| `configure_joint_command_interfaces` | Hardware registration |
| `configure_position_command_interface` | Hardware registration, used by `configure_joint_command_interfaces` |
| `configure_velocity_command_interface` | Hardware registration, used by `configure_joint_command_interfaces` |
| `configure_effort_command_interface` | Hardware registration, used by `configure_joint_command_interfaces` |
| `get_interfaces_in_order` | Model mapping, used by `get_ordered_command_interfaces` |

### `MujocoSystemInterface::register_transmissions`

| Helper | Category |
|---|---|
| `transmission_actuators_exist` | Hardware registration |
| `transmission_joint_interfaces_match` | Hardware registration |
| `make_transmission_joint_handles` | Hardware registration |
| `make_transmission_actuator_handles` | Hardware registration |
| `get_actuator_id` | Model mapping, used by `transmission_actuators_exist` |
| `add_items` | Model mapping, used by `make_transmission_joint_handles` |

Transmission plugin loading and `Transmission::configure` remain in the original method.

### `MujocoSystemInterface::initialize_initial_positions`

| Helper | Category |
|---|---|
| `copy_passive_joint_states` | Initial state |

### `MujocoSystemInterface::register_sensors`

| Helper | Category |
|---|---|
| `make_force_torque_sensor` | Hardware registration |
| `make_imu_sensor` | Hardware registration |
| `get_hardware_parameter_or` | Configuration, used by the sensor constructors |

### `MujocoSystemInterface::set_override_start_positions`

| Helper | Category |
|---|---|
| `load_initial_state_values` | Initial state |
| `initial_state_sizes_match` | Initial state |
| `copy_initial_state_to_data` | Initial state |

### `MujocoSystemInterface::set_initial_pose`

| Helper | Category |
|---|---|
| `apply_initial_pose` | Initial state |

### `MujocoSystemInterface::reset_simulation_state`

| Helper | Category |
|---|---|
| `reset_actuator_interfaces` | Initial state |
| `reset_joint_commands` | Initial state |

The existing actuator-to-joint transmission update remains between those two calls.

### `MujocoSystemInterface::load_mujoco_plugins`

| Helper | Category |
|---|---|
| `discover_plugin_names` | Plugin discovery |

Plugin loader creation, plugin initialization order, error handling, and instance storage remain in the original method.

## Build integration and verification

The helper implementations were added to the existing `mujoco_ros2_control` shared-library target in `CMakeLists.txt`.

Verification performed:

```text
ROS distribution: Jazzy
Command: colcon build --packages-select mujoco_ros2_control
Result: success; libmujoco_ros2_control.so and ros2_control_node linked and installed
```

The build used the already-built sibling message and plugin packages as underlays; neither sibling package was modified.
The package currently registers no CTest tests (`ctest --output-on-failure` reports `No tests were found`).
