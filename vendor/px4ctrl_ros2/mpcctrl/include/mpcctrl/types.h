// 独立于ROS的最小状态/轨迹/控制指令类型，替代rpg_mpc原版依赖的
// quadrotor_common::{QuadStateEstimate,Trajectory,TrajectoryPoint,ControlCommand}
// ——原版这几个类型内部各自#include了nav_msgs/Odometry.h、quadrotor_msgs/
// ControlCommand.h做ROS消息转换，本身不是纯POD。这里只保留mpc_controller.cpp
// 实际用到的字段，用Eigen类型直连，不耦合任何ROS消息类型，转换在mpcctrl_node.cpp
// 里做（订阅到的nav_msgs::msg::Odometry/quadrotor_msgs::msg::PositionCommand
// 现填到这些字段里，输出的ControlCommand再填进mavros_msgs::msg::AttitudeTarget）。
#pragma once

#include <Eigen/Eigen>
#include <vector>

namespace mpcctrl {

enum class ControlMode {
  NONE,
  BODY_RATES,
};

struct QuadState {
  double timestamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct TrajectoryPoint {
  double time_from_start{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  double heading{0.0};
};

struct Trajectory {
  std::vector<TrajectoryPoint> points;
};

struct ControlCommand {
  double timestamp{0.0};
  double expected_execution_time{0.0};
  bool armed{false};
  ControlMode control_mode{ControlMode::NONE};
  double collective_thrust{0.0};  // 质量归一化加速度(m/s^2)，不是0~1油门
  Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};

  void zero() {
    armed = false;
    control_mode = ControlMode::NONE;
    collective_thrust = 0.0;
    bodyrates.setZero();
    orientation = Eigen::Quaterniond::Identity();
  }
};

}  // namespace mpcctrl
