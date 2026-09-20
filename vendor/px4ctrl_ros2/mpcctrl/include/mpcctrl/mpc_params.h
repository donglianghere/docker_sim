// 移植自 uzh-rpg/rpg_mpc (GPLv3) include/rpg_mpc/mpc_params.h。
// 原版用quadrotor_common::getParam()+ros::NodeHandle读参数，这里改成
// rclcpp::Node的declare_parameter/get_parameter，参数名和默认值校验逻辑
// 原样保留，只是取参数的API换成ROS2的。
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "mpcctrl/mpc_wrapper.h"

namespace mpcctrl {

template <typename T>
class MpcParams {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcParams() :
    changed_(false),
    print_info_(false),
    state_cost_exponential_(0.0),
    input_cost_exponential_(0.0),
    max_bodyrate_xy_(0.0),
    max_bodyrate_z_(0.0),
    min_thrust_(0.0),
    max_thrust_(0.0),
    p_B_C_(Eigen::Matrix<T, 3, 1>::Zero()),
    q_B_C_(Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0)),
    Q_(Eigen::Matrix<T, kCostSize, kCostSize>::Zero()),
    R_(Eigen::Matrix<T, kInputSize, kInputSize>::Zero())
  {
  }

  ~MpcParams() {}

  bool loadParameters(rclcpp::Node* node)
  {
    #define DECLARE_GET(name, def) \
      node->declare_parameter<double>(#name, def); \
      double name = node->get_parameter(#name).as_double()

    // 状态代价
    DECLARE_GET(Q_pos_xy, 100.0);
    DECLARE_GET(Q_pos_z, 100.0);
    DECLARE_GET(Q_attitude, 50.0);
    DECLARE_GET(Q_velocity, 10.0);
    DECLARE_GET(Q_perception, 0.0);

    if (Q_pos_xy <= 0.0 || Q_pos_z <= 0.0 || Q_attitude <= 0.0 ||
        Q_velocity <= 0.0 || Q_perception < 0.0)
    {
      RCLCPP_ERROR(node->get_logger(), "MPC: State cost Q has negative enries!");
      return false;
    }

    // 输入代价
    DECLARE_GET(R_thrust, 1.0);
    DECLARE_GET(R_pitchroll, 10.0);
    DECLARE_GET(R_yaw, 1.0);

    if (R_thrust <= 0.0 || R_pitchroll <= 0.0 || R_yaw <= 0.0)
    {
      RCLCPP_ERROR(node->get_logger(), "MPC: Input cost R has negative enries!");
      return false;
    }

    Q_ = (Eigen::Matrix<T, kCostSize, 1>() <<
      Q_pos_xy, Q_pos_xy, Q_pos_z,
      Q_attitude, Q_attitude, Q_attitude, Q_attitude,
      Q_velocity, Q_velocity, Q_velocity,
      Q_perception, Q_perception).finished().asDiagonal();
    R_ = (Eigen::Matrix<T, kInputSize, 1>() <<
      R_thrust, R_pitchroll, R_pitchroll, R_yaw).finished().asDiagonal();

    DECLARE_GET(state_cost_exponential, 0.0);
    DECLARE_GET(input_cost_exponential, 0.0);
    state_cost_exponential_ = state_cost_exponential;
    input_cost_exponential_ = input_cost_exponential;

    DECLARE_GET(max_bodyrate_xy, 8.0);
    DECLARE_GET(max_bodyrate_z, 3.0);
    DECLARE_GET(min_thrust, 2.0);
    DECLARE_GET(max_thrust, 20.0);
    max_bodyrate_xy_ = max_bodyrate_xy;
    max_bodyrate_z_ = max_bodyrate_z;
    min_thrust_ = min_thrust;
    max_thrust_ = max_thrust;

    if (max_bodyrate_xy_ <= 0.0 || max_bodyrate_z_ <= 0.0 ||
        min_thrust_ <= 0.0 || max_thrust_ <= 0.0)
    {
      RCLCPP_ERROR(node->get_logger(), "MPC: All limits must be positive non-zero values!");
      return false;
    }

    node->declare_parameter<std::vector<double>>("p_B_C", std::vector<double>{0.0, 0.0, 0.0});
    auto p_B_C = node->get_parameter("p_B_C").as_double_array();
    if (p_B_C.size() == 3)
    {
      p_B_C_ = Eigen::Matrix<T, 3, 1>(p_B_C[0], p_B_C[1], p_B_C[2]);
    }
    node->declare_parameter<std::vector<double>>("q_B_C", std::vector<double>{1.0, 0.0, 0.0, 0.0});
    auto q_B_C = node->get_parameter("q_B_C").as_double_array();
    if (q_B_C.size() == 4)
    {
      q_B_C_ = Eigen::Quaternion<T>(q_B_C[0], q_B_C[1], q_B_C[2], q_B_C[3]);
    }

    node->declare_parameter<bool>("print_info", false);
    print_info_ = node->get_parameter("print_info").as_bool();
    if (print_info_)
      RCLCPP_INFO(node->get_logger(), "MPC: Informative printing enabled.");

    changed_ = true;

    #undef DECLARE_GET

    return true;
  }

  bool changed_;

  bool print_info_;

  T state_cost_exponential_;
  T input_cost_exponential_;

  T max_bodyrate_xy_;
  T max_bodyrate_z_;
  T min_thrust_;
  T max_thrust_;

  Eigen::Matrix<T, 3, 1> p_B_C_;
  Eigen::Quaternion<T> q_B_C_;

  Eigen::Matrix<T, kCostSize, kCostSize> Q_;
  Eigen::Matrix<T, kInputSize, kInputSize> R_;
};

} // namespace mpcctrl
