cmake_minimum_required(VERSION 3.16)
project(arm_behavior)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(arm_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(arm_behavior_node
  src/arm_action_server.cpp
  src/arm_behavior.cpp
  src/main.cpp
  src/motion_executor.cpp
  src/payload_service_server.cpp
  src/planning_scene_manager.cpp
  src/simulation_control.cpp
)
add_executable(environment_loader
  src/environment_loader.cpp
)
target_compile_features(arm_behavior_node PUBLIC cxx_std_17)
target_compile_features(environment_loader PUBLIC cxx_std_17)
target_include_directories(arm_behavior_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_behavior_node
  arm_interface
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
  std_srvs
)
ament_target_dependencies(environment_loader
  geometry_msgs
  moveit_msgs
  rclcpp
  shape_msgs
)

install(TARGETS arm_behavior_node environment_loader
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/compare_pose_frames.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
