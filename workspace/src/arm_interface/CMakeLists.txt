cmake_minimum_required(VERSION 3.8)
project(arm_interface)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CheckPoseReachability.action"
  "action/CloseGripper.action"
  "action/LiftGripper.action"
  "action/MoveArmToHomePose.action"
  "action/MoveArmToPose.action"
  "action/OpenGripper.action"
  "srv/AttachObjectToGripper.srv"
  "srv/DetachObjectFromGripper.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
