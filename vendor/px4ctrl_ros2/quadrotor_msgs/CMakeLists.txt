cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)  # 2026-09-20：Takeoff.action 需要

# px4ctrl只用到这三个消息（原Fast-Drone-250 quadrotor_msgs包里另外23个消息
# 跟px4ctrl无关，未移植）。PositionCommand.msg里的jerk字段必须保留——
# input.cpp的Command_Data_t::feed()要读msg.jerk.{x,y,z}，注意ego-planner-swarm
# 自带的ROS2版quadrotor_msgs/PositionCommand.msg缺这个字段，不能直接复用。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionCommand.msg"
  "msg/TakeoffLand.msg"
  "msg/Px4ctrlDebug.msg"
  "action/Takeoff.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
