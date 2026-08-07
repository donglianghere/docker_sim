#ifndef __INPUT_H
#define __INPUT_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

// 所有xxx_Data_t类各自持有一个独立的rclcpp::Clock(RCL_ROS_TIME)成员，
// 而不是像ROS1那样在任意地方直接调用静态的ros::Time::now()——rclcpp没有
// 这种全局静态时钟，必须挂在某个Clock对象上取时间。选RCL_ROS_TIME（而非
// RCL_SYSTEM_TIME）是为了在仿真里`use_sim_time:=true`时能跟随Gazebo仿真
// 时钟走，跟docker_sim里DLIO/mighty这些既有节点的时间基准保持一致；
// 代价是每个Data_t各自订阅一路/clock，多个独立RCL_ROS_TIME时钟在同一
// 进程内并存，有少量重复订阅开销，但没有正确性问题。
//
// 关键陷阱：rclcpp::Time两个不同clock_type的对象相减会在运行时抛异常，
// 所以这里全部固定用RCL_ROS_TIME（rcv_stamp的默认构造、feed()里取
// now()、PX4CtrlFSM.cpp里now_time的来源node_->get_clock()->now()）
// 必须全程保持一致，任何一处手滑用了RCL_SYSTEM_TIME都会在
// xxx_is_received()里(now_time - rcv_stamp)那行直接炸掉。

class RC_Data_t
{
public:
  double mode;
  double gear;
  double reboot_cmd;
  double last_mode;
  double last_gear;
  double last_reboot_cmd;
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];

  mavros_msgs::msg::RCIn msg;
  rclcpp::Time rcv_stamp;

  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;
  bool toggle_reboot;

  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
  static constexpr double DEAD_ZONE = 0.25;

  RC_Data_t();
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::msg::RCIn::ConstSharedPtr pMsg);
  bool is_received(const rclcpp::Time &now_time);

private:
  rclcpp::Clock clock_{RCL_ROS_TIME};
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
  Eigen::Vector3d w;

  nav_msgs::msg::Odometry msg;
  rclcpp::Time rcv_stamp;
  bool recv_new_msg;

  Odom_Data_t();
  void feed(nav_msgs::msg::Odometry::ConstSharedPtr pMsg);

private:
  rclcpp::Clock clock_{RCL_ROS_TIME};
};

class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;

  sensor_msgs::msg::Imu msg;
  rclcpp::Time rcv_stamp;

  Imu_Data_t();
  void feed(sensor_msgs::msg::Imu::ConstSharedPtr pMsg);

private:
  rclcpp::Clock clock_{RCL_ROS_TIME};
};

class State_Data_t
{
public:
  mavros_msgs::msg::State current_state;
  mavros_msgs::msg::State state_before_offboard;

  State_Data_t();
  void feed(mavros_msgs::msg::State::ConstSharedPtr pMsg);
};

class ExtendedState_Data_t
{
public:
  mavros_msgs::msg::ExtendedState current_extended_state;

  ExtendedState_Data_t();
  void feed(mavros_msgs::msg::ExtendedState::ConstSharedPtr pMsg);
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double yaw;
  double yaw_rate;

  quadrotor_msgs::msg::PositionCommand msg;
  rclcpp::Time rcv_stamp;

  Command_Data_t();
  void feed(quadrotor_msgs::msg::PositionCommand::ConstSharedPtr pMsg);

private:
  rclcpp::Clock clock_{RCL_ROS_TIME};
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::msg::BatteryState msg;
  rclcpp::Time rcv_stamp;

  Battery_Data_t();
  void feed(sensor_msgs::msg::BatteryState::ConstSharedPtr pMsg);

private:
  rclcpp::Clock clock_{RCL_ROS_TIME};
};

class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};
  uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination

  quadrotor_msgs::msg::TakeoffLand msg;
  rclcpp::Time rcv_stamp;

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::msg::TakeoffLand::ConstSharedPtr pMsg);

private:
  rclcpp::Clock clock_{RCL_ROS_TIME};
};

#endif
