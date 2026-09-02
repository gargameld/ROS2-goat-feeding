
#include "mujoco_ros2_control/system_interface/state_reading.hpp"

namespace mujoco_ros2_control
{

void read_actuator_states(const mjData* control_data, std::vector<MuJoCoActuatorData>& actuators)
{
  for (auto& actuator_state : actuators)
  {
    actuator_state.position_interface.state_ = control_data->qpos[actuator_state.mj_pos_adr];
    actuator_state.velocity_interface.state_ = control_data->qvel[actuator_state.mj_vel_adr];
    actuator_state.effort_interface.state_ = control_data->qfrc_actuator[actuator_state.mj_vel_adr];
  }
}

void read_imu_states(const mjData* control_data, std::vector<IMUSensorData>& sensors)
{
  for (auto& data : sensors)
  {
    data.orientation.data.w() = control_data->sensordata[data.orientation.mj_sensor_index];
    data.orientation.data.x() = control_data->sensordata[data.orientation.mj_sensor_index + 1];
    data.orientation.data.y() = control_data->sensordata[data.orientation.mj_sensor_index + 2];
    data.orientation.data.z() = control_data->sensordata[data.orientation.mj_sensor_index + 3];

    data.angular_velocity.data.x() = control_data->sensordata[data.angular_velocity.mj_sensor_index];
    data.angular_velocity.data.y() = control_data->sensordata[data.angular_velocity.mj_sensor_index + 1];
    data.angular_velocity.data.z() = control_data->sensordata[data.angular_velocity.mj_sensor_index + 2];

    data.linear_acceleration.data.x() = control_data->sensordata[data.linear_acceleration.mj_sensor_index];
    data.linear_acceleration.data.y() = control_data->sensordata[data.linear_acceleration.mj_sensor_index + 1];
    data.linear_acceleration.data.z() = control_data->sensordata[data.linear_acceleration.mj_sensor_index + 2];
  }
}

}  // namespace mujoco_ros2_control
