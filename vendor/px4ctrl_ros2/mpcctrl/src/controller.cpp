// mpcctrl包的核心改动文件。原版px4ctrl的LinearControl::calculateControl()
// 是一段线性化的P+D近似；so3ctrl换成了SO3几何控制律；这里换成
// uzh-rpg/rpg_mpc移植版(mpcctrl::MpcController，见mpc_controller.h/.cpp，
// GPLv3)的模型预测控制——用当前state_estimate+一段参考轨迹，滚动求解
// 一段2秒预测时域(见mpc_wrapper.h的ACADO_N=20、dt=0.1s)，取第一步的
// body_rate+collective_thrust作为这一拍的实际输出。
//
// 保留的部分：thr2acc_在线自标定(RLS)——跟px4ctrl/so3ctrl原版一模一样的
// 算法，MPC给出的collective_thrust是质量归一化推力(m/s^2)，跟px4ctrl原版
// des_acc(2)是同一个量纲，复用这套自标定换算成[0,1]归一化油门。
//
// 明确没做的事：odom"内容新鲜度"外推（DEBUG_JOURNAL 2026-09-13"px4ctrl
// 拿到的里程计有延迟"那条记录提出、在独立的mpcctrl_node.cpp第一版里做过
// 实验性实现的Plan B）——用户明确说"先不管这个新鲜度了"，这次接入FSM
// 只做状态机功能对齐，新鲜度增强留到以后单独做，不在这次范围内。
#include "mpcctrl/controller.h"

using namespace std;

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param, rclcpp::Node *node) : param_(param)
{
  if (!mpc_params_.loadParameters(node))
  {
    RCLCPP_FATAL(node->get_logger(), "[mpcctrl] MPC参数加载失败，节点无法启动");
    throw std::runtime_error("mpcctrl: failed to load MPC parameters");
  }
  mpc_ = std::make_unique<mpcctrl::MpcController<double>>(mpc_params_);
  resetThrustMapping();
}

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::msg::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u)
{
  mpcctrl::QuadState state;
  state.position = odom.p;
  state.velocity = odom.v;
  state.orientation = odom.q;

  mpcctrl::TrajectoryPoint tp;
  tp.position = des.p;
  tp.velocity = des.v;
  tp.acceleration = des.a;
  tp.heading = des.yaw;
  // 注意：des.q在这个项目里对CMD_CTRL/AUTO_HOVER/AUTO_TAKEOFF/AUTO_LAND
  // 这几个真正会被MPC用到的状态都是Desired_State_t()默认构造出来的，
  // Eigen::Quaterniond默认构造不是单位阵、是未初始化内存——px4ctrl原版
  // 自己的控制律从没用过des.q(它用的是本地重新算的roll/pitch/yaw)，这
  // 是上游代码里一直存在、但从未暴露的一个"看起来能用、实际是垃圾值"的
  // 字段。这里故意不读des.q，tp.orientation保持types.h里的默认单位四元数，
  // 让参考姿态完全由tp.heading(des.yaw)决定——这跟rpg_mpc原版公式
  // `q_orientation = points.front().orientation * q_heading`在
  // orientation=单位阵时退化出的结果完全一致，是有意为之，不是漏填。
  // tp.bodyrates也保持默认零——PositionCommand本身不携带角速度参考。

  mpcctrl::Trajectory traj;
  traj.points.push_back(tp);

  const mpcctrl::ControlCommand cmd = mpc_->run(state, traj, mpc_params_);

  u.q = cmd.orientation;
  u.bodyrates = cmd.bodyrates;
  u.thrust = computeDesiredCollectiveThrustSignal(cmd.collective_thrust);

  //used for debug
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);

  debug_msg_.des_a_x = des.a(0);
  debug_msg_.des_a_y = des.a(1);
  debug_msg_.des_a_z = des.a(2);

  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();

  debug_msg_.des_thr = u.thrust;

  debug_msg_.fb_rate_x = u.bodyrates.x();
  debug_msg_.fb_rate_y = u.bodyrates.y();
  debug_msg_.fb_rate_z = u.bodyrates.z();

  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<rclcpp::Time, double>(clock_.now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage
*/
double
LinearControl::computeDesiredCollectiveThrustSignal(double collective_thrust)
{
  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = collective_thrust / thr2acc_;

  return throttle_percentage;
}

bool
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  rclcpp::Time t_now = clock_.now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<rclcpp::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).seconds();
    if (time_passed > 0.045) // 45ms
    {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();

    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;

    return true;
  }
  return false;
}

void
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
  mpc_->resetWarmStart();
}
