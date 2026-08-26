cmake_minimum_required(VERSION 3.16)

project(mujoco_ros2_control)

find_package(ros2_control_cmake REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(glfw3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(Threads REQUIRED)
find_package(backward_ros REQUIRED)
find_package(tinyxml2_vendor QUIET)
find_package(TinyXML2 REQUIRED)

# Link MuJoCo via the vendor package
find_package(mujoco_vendor REQUIRED)
set(MUJOCO_LIB mujoco::mujoco)

# The vendor package may be installed in different locations depending on if it
# was installed with apt vs pixi. So resolve the install root and use that to
# locate the relevant header files for this project.
# TODO: It would easier if the vendor package exported the variables that we need here.
get_filename_component(MUJOCO_PREFIX "${mujoco_vendor_DIR}/../../.." ABSOLUTE)
set(MUJOCO_ROOT "${MUJOCO_PREFIX}/opt/mujoco_vendor")
if(NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
  set(MUJOCO_ROOT "${MUJOCO_PREFIX}")
endif()
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT}/include")
set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT}/simulate")
if(NOT EXISTS "${MUJOCO_SIMULATE_DIR}/simulate.h")
  set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT}/include/simulate")
endif()
# So downstream packages that include our headers can find glfw_adapter.h etc.
set(MUJOCO_INSTALL_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}")
set(MUJOCO_INSTALL_SIMULATE_DIR "${MUJOCO_SIMULATE_DIR}")

# Fetch lodepng dependency, if not available
find_package(lodepng QUIET)
if(NOT TARGET lodepng)
  include(FetchContent)
  if(NOT DEFINED MUJOCO_DEP_VERSION_lodepng)
    message(WARNING "MUJOCO_DEP_VERSION_lodepng not exported by mujoco_vendor; fetching lodepng HEAD")
  endif()
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    set_target_properties(lodepng PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif()
  install(TARGETS lodepng
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
endif()

# We don't care about warnings from the external sources
set(MUJOCO_SILENCE_COMPILER_WARNINGS
  -Wno-missing-field-initializers
  -Wno-unused-parameter
  -Wno-sign-compare
  -Wno-psabi
)

# Check if MuJoCo's simulate application is available for linking. The github tarfiles packaged
# by the vendor package has the simulate application compiled with -flto=auto, so we cannot
# actually link against it as normal symbols are not available. If that is the case, we must
# compile the application from source. For the conda installed version, we can link directly
# against the package. We create a `mujoco_simulate` library in both cases.
find_library(mujoco_simulate_lib
  NAMES simulate
  PATHS "${MUJOCO_ROOT}/lib"
  NO_DEFAULT_PATH
)
if(mujoco_simulate_lib AND NOT EXISTS "${MUJOCO_SIMULATE_DIR}/simulate.cc")
  # Conda installed package does not have source, so link against the library directly
  message(STATUS "Linking against the mujoco simulate application directly")
  add_library(mujoco_simulate STATIC IMPORTED)
  set_target_properties(mujoco_simulate PROPERTIES
    IMPORTED_LOCATION "${mujoco_simulate_lib}"
  )
  target_include_directories(mujoco_simulate INTERFACE ${MUJOCO_SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(mujoco_simulate INTERFACE ${MUJOCO_LIB} glfw)
else()
  message(STATUS "No linkable simulate application available, compiling from source")
  add_library(platform_ui_adapter OBJECT)
  set_target_properties(platform_ui_adapter PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  )
  target_sources(
    platform_ui_adapter
    PRIVATE ${MUJOCO_SIMULATE_DIR}/glfw_adapter.h ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.h ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.h
    PRIVATE ${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc
  )
  target_include_directories(platform_ui_adapter PUBLIC ${MUJOCO_SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(platform_ui_adapter PRIVATE ${MUJOCO_LIB} glfw)
  target_compile_options(platform_ui_adapter PRIVATE ${MUJOCO_SILENCE_COMPILER_WARNINGS})
  add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

  add_library(mujoco_simulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
  set_target_properties(mujoco_simulate PROPERTIES
    OUTPUT_NAME simulate
    POSITION_INDEPENDENT_CODE ON
  )
  target_sources(mujoco_simulate
    PRIVATE ${MUJOCO_SIMULATE_DIR}/simulate.h
    PRIVATE ${MUJOCO_SIMULATE_DIR}/simulate.cc ${MUJOCO_SIMULATE_DIR}/array_safety.h
  )
  target_include_directories(mujoco_simulate PUBLIC ${MUJOCO_SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(mujoco_simulate PRIVATE lodepng mujoco::platform_ui_adapter ${MUJOCO_LIB})
  target_compile_options(mujoco_simulate PRIVATE ${MUJOCO_SILENCE_COMPILER_WARNINGS})

  # We set these after creating the targets to avoid propagating them to other third-party targets
  set_compiler_options()
  export_windows_symbols()
  install(TARGETS mujoco_simulate
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endif()

# MuJoCo ROS 2 control system interface (this package)
add_library(mujoco_ros2_control SHARED
    # ros2_control system interface: bridges the URDF/ros2_control model to MuJoCo
    src/mujoco_system_interface.cpp
    src/system_interface/simulation_configuration.cpp
    src/system_interface/mujoco_model_validation.cpp
    src/system_interface/mujoco_actuator_discovery.cpp
    src/system_interface/joint_command_setup.cpp
    src/system_interface/imu_sensor_setup.cpp
    src/system_interface/sensor_registration.cpp
    src/system_interface/interface_export.cpp
    src/system_interface/initial_state.cpp
    src/system_interface/joint_actuator_mapping.cpp
    src/system_interface/command_mode_switching.cpp
    src/system_interface/state_reading.cpp
    src/system_interface/control_plugin_loader.cpp
    # simulation host: owns the MuJoCo model/data, physics thread and /clock
    src/simulation/mujoco_simulation.cpp
    src/simulation/physics_loop.cpp
    src/simulation/mujoco_model_loader.cpp
    src/simulation/mujoco_extension_loader.cpp
    src/simulation/mujoco_simulation_clock.cpp
    src/simulation/physics_loop_synchronizer.cpp
    # sensors: offscreen camera rendering into point clouds
    src/sensors/mujoco_cameras.cpp
    src/sensors/depth_to_pointcloud.cpp
)
# Ensure MuJoCo library can be found at runtime regardless of how it was installed above.
# mujoco_vendor installs libmujoco.so under opt/mujoco_vendor/lib/ relative to the
# install prefix, so that directory must be in the RPATH alongside the standard lib/.
set_target_properties(mujoco_ros2_control PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../opt/mujoco_vendor/lib;${CMAKE_INSTALL_PREFIX}/lib"
)
# MuJoCo has TinyXML2 statically linked, which conflicts with the system TinyXML2
# used by FastDDS. We need to ensure the system TinyXML2 is loaded first.
# Force tinyxml2 to appear first in the DT_NEEDED entries by using LINK_OPTIONS.
# TODO: Remove this after the following is resolved and the next ROS sync
# https://github.com/pal-robotics/mujoco_vendor/issues/2
target_link_options(mujoco_ros2_control PUBLIC
  "LINKER:--push-state,--no-as-needed"
  "$<TARGET_LINKER_FILE:tinyxml2::tinyxml2>"
  "LINKER:--pop-state"
)
target_link_libraries(mujoco_ros2_control
PUBLIC
  ament_index_cpp::ament_index_cpp
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${controller_manager_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  pluginlib::pluginlib
  fmt::fmt
  ${MUJOCO_LIB}
  Eigen3::Eigen
PRIVATE
  mujoco_simulate
  Threads::Threads
  glfw
)
target_include_directories(mujoco_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_SIMULATE_DIR}>
  $<TARGET_PROPERTY:mujoco_ros2_control_plugins::mujoco_ros2_control_plugins,INTERFACE_INCLUDE_DIRECTORIES>
)
# Mark MuJoCo source as system to avoid compiler warnings
target_include_directories(mujoco_ros2_control SYSTEM PUBLIC
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MUJOCO_SIMULATE_DIR}>
)

# Note: mujoco::mujoco is an IMPORTED target owned by mujoco_vendor and cannot be
# co-installed here. Downstream packages must find_package(mujoco_vendor) directly,
# which is ensured via ament_export_dependencies below.
install(TARGETS mujoco_ros2_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Install the ROS 2 control node
add_executable(ros2_control_node src/mujoco_ros2_control_node.cpp)
target_link_libraries(ros2_control_node PUBLIC
  controller_manager::controller_manager
  ${controller_manager_msgs_TARGETS}
)
install(
  TARGETS ros2_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install python tools
install(PROGRAMS
  scripts/find_missing_inertias.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

pluginlib_export_plugin_description_file(hardware_interface mujoco_system_interface_plugin.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  mujoco_ros2_control_plugins
  pluginlib
  Eigen3
  mujoco_vendor
)
ament_package()
