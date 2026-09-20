// mpcctrl包的核心改动文件：外层类型（Desired_State_t/Controller_Output_t/
// 类名LinearControl/成员函数签名）全部保持跟px4ctrl/so3ctrl原版一致，这样
// PX4CtrlFSM.h/.cpp可以逐字复制、完全不用碰（PX4CtrlFSM.h里
// `LinearControl &controller`这个类型名是唯一的耦合点）。真正变了的是类
// 内部实现：不再是px4ctrl原版的P+D线性近似，也不是so3ctrl的SO3几何控制律，
// 换成uzh-rpg/rpg_mpc移植版(mpcctrl::MpcController，见mpc_controller.h，
// GPLv3)的模型预测控制。类名"LinearControl"因此继续沿用so3ctrl留下的
// "名不副实"历史包袱，不是疏忽。
//
// 跟so3ctrl的关键差异：so3ctrl的SO3Control只需要构造时设一次mass/gravity/
// max_angle，不需要访问rclcpp::Node；这里的MPC核心(mpcctrl::MpcController)
// 有自己独立的一整套代价矩阵/限幅参数(mpcctrl::MpcParams)，需要在构造时
// 用rclcpp::Node::declare_parameter/get_parameter读取——所以LinearControl
// 的构造函数签名比so3ctrl多了一个rclcpp::Node*参数。这不影响PX4CtrlFSM.h
// （FSM从不构造LinearControl，只在mpcctrl_node.cpp里构造一次），零耦合。
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/msg/attitude_target.hpp>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>
#include <memory>
#include <queue>

#include "mpcctrl/input.h"
#include "mpcctrl/mpc_controller.h"
#include "mpcctrl/mpc_params.h"
#include "mpcctrl/types.h"
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
  // 比so3ctrl/px4ctrl原版多一个node参数，专门用来加载MPC自己的代价矩阵/
  // 限幅参数(mpcctrl::MpcParams::loadParameters)——PX4CtrlFSM.h从不构造
  // LinearControl，这个签名改动不影响FSM代码。
  LinearControl(Parameter_t &, rclcpp::Node *node);
  quadrotor_msgs::msg::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu,
      Controller_Output_t &u);
  bool estimateThrustModel(const Eigen::Vector3d &est_a,
      const Parameter_t &param);
  // 除了原有的thr2acc_/P_在线自标定复位，这一版还顺带让MPC下一次求解
  // 强制从悬停初始猜测重新收敛(mpc_.resetWarmStart())——不需要在
  // PX4CtrlFSM.cpp里另外新增调用点，直接复用它已有的两处
  // resetThrustMapping()调用（MANUAL_CTRL→AUTO_HOVER、进入AUTO_TAKEOFF）。
  void resetThrustMapping(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  mpcctrl::MpcParams<double> mpc_params_;
  // unique_ptr而不是直接持有MpcController<double>实例：MpcController内部有
  // std::thread成员(preparation_thread_)，一旦某个类声明了自己的析构函数
  // (MpcController为了兜底join这个线程声明了)，编译器就不会再自动生成
  // 移动赋值——而mpc_params_必须先在构造函数体内(不是初始化列表)用node读完
  // 参数才能构造MpcController，没法在类的成员初始化阶段直接构造它，只能
  // 用unique_ptr延迟到构造函数体里再make_unique，规避掉移动/拷贝赋值的
  // 问题。
  std::unique_ptr<mpcctrl::MpcController<double>> mpc_;
  quadrotor_msgs::msg::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<rclcpp::Time, double>> timed_thrust_;

  // Thrust-accel mapping params——跟px4ctrl/so3ctrl原版一样的在线RLS
  // 自标定。MPC给出的collective_thrust是质量归一化推力(m/s^2)，跟px4ctrl
  // 原版des_acc(2)是同一个量纲(见mpcctrl::ControlCommand::collective_thrust
  // 的定义)，复用这套自标定做换算成[0,1]归一化油门。
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  rclcpp::Clock clock_{RCL_ROS_TIME};

  double computeDesiredCollectiveThrustSignal(double collective_thrust);
  double fromQuaternion2yaw(Eigen::Quaterniond q);
};


#endif
