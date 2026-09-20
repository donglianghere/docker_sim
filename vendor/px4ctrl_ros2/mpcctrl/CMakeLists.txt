cmake_minimum_required(VERSION 3.8)
project(mpcctrl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
# qpOASES/ACADO生成的C代码里有不少形参未使用/隐式转换告警，跟px4ctrl的
# -Wall -g不同，这里放宽一点，不然build log会被第三方代码的警告刷屏。
set(CMAKE_C_FLAGS_RELEASE "-O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# ARM NEON优化——照搬rpg_mpc原版CMakeLists里的判断，Jetson Orin NX是armv8/
# aarch64不是armv7l，这条分支实际不会命中，保留只是为了跟上游行为一致、
# 万一以后要在armv7l设备上编译也不用再补。
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
endif()

# 求解器：qpOASES(C++) + ACADO codegen(C)，原样照搬自rpg_mpc，零ROS依赖。
add_library(mpc_solver STATIC
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
  model/quadrotor_mpc_codegen/acado_integrator.c
  model/quadrotor_mpc_codegen/acado_solver.c
  model/quadrotor_mpc_codegen/acado_auxiliary_functions.c
)
target_include_directories(mpc_solver PUBLIC
  model/quadrotor_mpc_codegen/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
)

add_library(mpc_wrapper
  src/mpc_wrapper.cpp
)
target_include_directories(mpc_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(mpc_wrapper mpc_solver)

add_library(mpc_controller
  src/mpc_controller.cpp
)
target_include_directories(mpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(mpc_controller mpc_wrapper)
ament_target_dependencies(mpc_controller rclcpp)

# FSM/input/param逐字复制自px4ctrl（见NOTICE.md），controller.cpp是本包
# 唯一替换掉的控制律实现，调用上面的mpc_controller/mpc_wrapper/mpc_solver。
add_executable(mpcctrl_node
  src/mpcctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)
target_include_directories(mpcctrl_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(mpcctrl_node mpc_controller mpc_wrapper mpc_solver)
ament_target_dependencies(mpcctrl_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  quadrotor_msgs
  uav_utils
)

install(TARGETS mpcctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
