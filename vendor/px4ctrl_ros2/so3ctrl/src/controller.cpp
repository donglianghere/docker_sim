// so3ctrl包的核心改动文件。原版px4ctrl的LinearControl::calculateControl()
// 是一段线性化的P+D近似（roll/pitch直接从des_acc的x/y分量除以g反解，
// 欧拉角合成四元数，没有积分项、没有倾角限幅）。这里换成
// KumarRobotics/kr_mav_control（BSD-3-Clause，SO3Control.hpp/cpp，未改
// 动，来源见该目录文件头注释）的SO3几何控制律：位置环P+D+世界系/机体系
// 双积分，姿态是从期望合力方向+期望偏航构造的精确非线性旋转矩阵（不是
// 线性近似），额外带最大倾角限幅。
//
// 保留的部分：thr2acc_在线自标定（RLS，估计"归一化油门→世界系Z轴加速度"
// 的映射关系）——SO3Control给的是牛顿力，不是px4ctrl这套mavros接口要的
// [0,1]归一化油门，复用这套自标定做换算，避免引入kr_mav_control原生那套
// 需要额外测力台标定的kf/lin_cof_a/lin_int_b物理模型（该模型细节见
// docker_sim/DEBUG_JOURNAL.md相关记录）。
//
// 明确没做的事：SO3Control算出来的角速度(getComputedAngularVelocity())
// 这版没有接进u.bodyrates——use_bodyrate_ctrl依然是false（见
// config/ctrl_param_fpv.yaml），走的是publish_attitude_ctrl()那条路径，
// 角速度这个量目前不会被使用。要不要切到bodyrate模式是另一个需要单独
// 验证的行为改动，不在这次范围内。
#include "controller.h"

using namespace std;

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  // SO3Control这几个setter只需要在构造时设一次：质量/重力/最大倾角在
  // 飞行过程中不会变(mass_目前也没有热更新入口，跟px4ctrl原版一致)。
  // max_angle复用了Parameter_t里本来就有、但px4ctrl原版从未真正用过的
  // 那个字段（PX4CtrlParam.cpp里只转换成弧度、没有任何地方读它）——
  // 这里终于把它接上了。
  so3_control_.setMass(static_cast<float>(param_.mass));
  so3_control_.setGravity(static_cast<float>(param_.gra));
  so3_control_.setMaxTiltAngle(static_cast<float>(param_.max_angle));
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
  // odom/des都是double精度(Vector3d/Quaterniond)，SO3Control全套是float
  // 精度(Vector3f/Quaternionf)——这是这次移植唯一需要做的glue，.cast<>()
  // 逐个转换，不涉及任何数值算法上的改动。
  so3_control_.setPosition(odom.p.cast<float>());
  so3_control_.setVelocity(odom.v.cast<float>());
  so3_control_.setCurrentOrientation(odom.q.cast<float>());

  const Eigen::Vector3f kx(static_cast<float>(param_.gain.Kp0), static_cast<float>(param_.gain.Kp1), static_cast<float>(param_.gain.Kp2));
  const Eigen::Vector3f kv(static_cast<float>(param_.gain.Kv0), static_cast<float>(param_.gain.Kv1), static_cast<float>(param_.gain.Kv2));
  const Eigen::Vector3f ki(static_cast<float>(param_.gain.Ki0), static_cast<float>(param_.gain.Ki1), static_cast<float>(param_.gain.Ki2));
  const Eigen::Vector3f kib(static_cast<float>(param_.gain.Kib0), static_cast<float>(param_.gain.Kib1), static_cast<float>(param_.gain.Kib2));

  so3_control_.calculateControl(des.p.cast<float>(), des.v.cast<float>(), des.a.cast<float>(), des.j.cast<float>(),
                                 static_cast<float>(des.yaw), static_cast<float>(des.yaw_rate), kx, kv, ki, kib);

  const Eigen::Vector3f &force = so3_control_.getComputedForce();
  const Eigen::Quaternionf &q_des = so3_control_.getComputedOrientation();

  u.q = q_des.cast<double>();
  // 推力换算：force.z()/mass是"世界系Z轴期望加速度"，跟px4ctrl原版
  // des_acc(2)是同一个量纲——thr2acc_本来就是照这个量在线标定的，这里
  // 不是新公式，只是这个z轴分量现在来自SO3Control内部(已经过倾角限幅
  // 处理)算出来的合力，不是重新算一遍。
  u.thrust = computeDesiredCollectiveThrustSignal(static_cast<double>(force.z()) / param_.mass);
  // u.bodyrates不设，保持Controller_Output_t默认构造的零值——见文件头
  // "明确没做的事"。

  //used for debug
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);

  debug_msg_.des_a_x = static_cast<double>(force.x()) / param_.mass;
  debug_msg_.des_a_y = static_cast<double>(force.y()) / param_.mass;
  debug_msg_.des_a_z = static_cast<double>(force.z()) / param_.mass;

  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();

  debug_msg_.des_thr = u.thrust;

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
LinearControl::computeDesiredCollectiveThrustSignal(double des_acc_z)
{
  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc_z / thr2acc_;

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
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
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
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
  so3_control_.resetIntegrals();
}
