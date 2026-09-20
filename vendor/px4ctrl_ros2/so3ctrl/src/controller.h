// so3ctrl包的核心改动文件：外层类型（Desired_State_t/Controller_Output_t/
// 类名LinearControl/成员函数签名）全部保持跟px4ctrl原版一致，这样
// PX4CtrlFSM.h/.cpp可以逐字复制、完全不用碰（PX4CtrlFSM.h里
// `LinearControl &controller`这个类型名是唯一的耦合点，类名不改，那个
// 文件就不用改）。真正变了的是类内部实现：不再是px4ctrl原版的P+D线性近似，
// 换成KumarRobotics kr_mav_control（BSD-3-Clause）的SO3Control几何控制律
// ——细节见controller.cpp。类名"LinearControl"因此变得名不副实（内部早就
// 不是线性控制器了），这是刻意保留的历史包袱，不是疏忽。
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/msg/attitude_target.hpp>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>
#include <queue>

#include "input.h"
#include "SO3Control.hpp"
#include <Eigen/Dense>

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};


class LinearControl
{
public:
  LinearControl(Parameter_t &);
  quadrotor_msgs::msg::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu,
      Controller_Output_t &u);
  bool estimateThrustModel(const Eigen::Vector3d &est_v,
      const Parameter_t &param);
  // 除了原有的thr2acc_/P_在线自标定复位，这一版还顺带清空SO3Control的
  // 位置积分项(so3_control_.resetIntegrals())——不需要在PX4CtrlFSM.cpp里
  // 另外新增调用点，直接复用它已有的两处resetThrustMapping()调用
  // （MANUAL_CTRL→AUTO_HOVER、进入AUTO_TAKEOFF），这两个时机同时清空
  // 推力估计和位置积分本来就是同一件事："这是一次全新的起飞/悬停，之前
  // 累积的状态不该带过来"。
  void resetThrustMapping(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::msg::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<rclcpp::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params——跟px4ctrl原版一样的在线RLS自标定
  // （见controller.cpp的estimateThrustModel），SO3Control算出来的是牛顿
  // 力，不是px4ctrl这套要的[0,1]归一化油门，复用这套自标定做换算，不用
  // 引入kr_mav_control原生那套需要测力台标定的kf/lin_cof_a/lin_int_b模型。
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  rclcpp::Clock clock_{RCL_ROS_TIME};

  // kr_mav_control(KumarRobotics, BSD-3-Clause)的SO3几何位置环控制律，
  // 真正算力/姿态/角速度的地方，见SO3Control.hpp/.cpp。
  SO3Control so3_control_;

  double computeDesiredCollectiveThrustSignal(double des_acc_z);
  double fromQuaternion2yaw(Eigen::Quaterniond q);
};


#endif
