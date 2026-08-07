/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/msg/attitude_target.hpp>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>
#include <queue>

#include "input.h"
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
  void resetThrustMapping(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::msg::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<rclcpp::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  // ROS1版本这里到处直接调静态ros::Time::now()，ROS2没有等价的全局静态时钟，
  // 这个类自己挂一个RCL_ROS_TIME时钟——跟input.h里每个xxx_Data_t类的做法
  // 一致，同样是为了在push进timed_thrust_队列的时间戳和estimateThrustModel()
  // 里取当前时刻两边用的是同一种clock_type，避免rclcpp::Time相减时因
  // clock_type不一致抛异常。
  rclcpp::Clock clock_{RCL_ROS_TIME};

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  double fromQuaternion2yaw(Eigen::Quaterniond q);
};


#endif
