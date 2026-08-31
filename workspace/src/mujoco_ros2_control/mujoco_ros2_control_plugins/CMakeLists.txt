cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_control_plugins)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Attempt to link MuJoCo via the vendor package, if available
find_package(mujoco_vendor QUIET)
if(mujoco_vendor_FOUND)
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${mujoco_vendor_INCLUDE_DIRS})
else()
  # Fallback to system MuJoCo or other methods if needed
  message(FATAL_ERROR "mujoco_vendor not found!")
endif()


# Build the plugin library (all plugins compiled into one shared library)
add_library(mujoco_ros2_control_plugins SHARED
  src/plugin_parameters.cpp
  src/mujoco_3d_lidar_plugin.cpp
  src/state_capture/state_capture_plugin.cpp
  src/state_capture/state_capture_consumer.cpp
  src/lock_robot_base_plugin.cpp
  src/obstacle_control/obstacle_control_plugin.cpp
  src/food_control/food_control_plugin.cpp
  src/food_control/food_management.cpp
  src/simulation_state_provider/simulation_state_provider_plugin.cpp
)
target_include_directories(mujoco_ros2_control_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Mark MuJoCo source as system to avoid compiler warnings
target_include_directories(mujoco_ros2_control_plugins SYSTEM PUBLIC
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
)
target_link_libraries(mujoco_ros2_control_plugins PUBLIC
  ${MUJOCO_LIB}
  rclcpp::rclcpp
  realtime_tools::realtime_tools
  pluginlib::pluginlib
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${mujoco_ros2_control_msgs_TARGETS}
  ${std_srvs_TARGETS}
)
# Install the plugin library
install(TARGETS mujoco_ros2_control_plugins
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install runtime plugin configuration files.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export plugin description
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins mujoco_ros2_control_plugins.xml)

ament_export_include_directories(include)
ament_export_dependencies(
  mujoco_vendor
  rclcpp
  realtime_tools
  pluginlib
  geometry_msgs
  sensor_msgs
  mujoco_ros2_control_msgs
  std_srvs
  ros2_control_cmake
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
