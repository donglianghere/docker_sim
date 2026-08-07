#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <rclcpp/rclcpp.hpp>
#include <cassert>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include "input.h"
// #include "ThrustCurve.h"
#include "controller.h"

struct AutoTakeoffLand_t
{
	bool landed{true};
	// 默认成员初始化必须显式指定RCL_ROS_TIME clock_type——rclcpp::Time的
	// 默认构造函数走的是RCL_SYSTEM_TIME，如果这里漏掉，后面跟node_->now()
	// （RCL_ROS_TIME）相减会在运行时直接抛异常，是ROS2移植里最容易踩的坑
	// 之一，input.h开头也有同样的说明。
	rclcpp::Time toggle_takeoff_land_time{0, 0, RCL_ROS_TIME};
	std::pair<bool, rclcpp::Time> delay_trigger{std::pair<bool, rclcpp::Time>(false, rclcpp::Time(0, 0, RCL_ROS_TIME))};
	Eigen::Vector4d start_pose;

	static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motors idle running for 3 seconds before takeoff
	static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
};

class PX4CtrlFSM
{
public:
	Parameter_t &param;

	RC_Data_t rc_data;
	State_Data_t state_data;
	ExtendedState_Data_t extended_state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Command_Data_t cmd_data;
	Battery_Data_t bat_data;
	Takeoff_Land_Data_t takeoff_land_data;

	LinearControl &controller;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traj_start_trigger_pub;
	rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr ctrl_FCU_pub;
	rclcpp::Publisher<quadrotor_msgs::msg::Px4ctrlDebug>::SharedPtr debug_pub; //debug
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_FCU_mode_srv;
	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_srv;
	rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr reboot_FCU_srv;

	quadrotor_msgs::msg::Px4ctrlDebug debug_msg; //debug

	Eigen::Vector4d hover_pose;
	// 同样必须显式给RCL_ROS_TIME，理由见AutoTakeoffLand_t::toggle_takeoff_land_time
	// 的注释——这个成员实际运行路径上总是先经set_hov_with_odom()赋值过一次才会被
	// set_hov_with_rc()拿去做减法，此处默认值本来不会被用到，但仍按"任何
	// rclcpp::Time成员一律显式RCL_ROS_TIME"的统一约定初始化，避免以后重构时
	// 引入真正会触发的clock_type不一致异常。
	rclcpp::Time last_set_hover_pose_time{0, 0, RCL_ROS_TIME};

	enum State_t
	{
		MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER, // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// px4ctrl is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND
	};

	// node是原ROS1版本没有的新参数——服务调用(toggle_offboard_mode等)在ROS2下
	// 要用rclcpp::spin_until_future_complete()异步等待响应，这个函数要求传入
	// rclcpp::node_interfaces::NodeBaseInterface::SharedPtr（通过
	// node->get_node_base_interface()拿到，node本身不需要是shared_ptr持有，
	// 存raw指针即可），所以FSM需要一份指向宿主节点的指针；同时rclcpp的所有
	// RCLCPP_INFO/WARN/ERROR日志宏也要挂在某个logger上，一并用node->get_logger()。
	PX4CtrlFSM(Parameter_t &, LinearControl &, rclcpp::Node *node);
	void process();
	bool rc_is_received(const rclcpp::Time &now_time);
	bool cmd_is_received(const rclcpp::Time &now_time);
	bool odom_is_received(const rclcpp::Time &now_time);
	bool imu_is_received(const rclcpp::Time &now_time);
	bool bat_is_received(const rclcpp::Time &now_time);
	bool recv_new_odom();
	State_t get_state() { return state; }
	bool get_landed() { return takeoff_land.landed; }

private:
	rclcpp::Node *node_;
	State_t state; // Should only be changed in PX4CtrlFSM::process() function!
	AutoTakeoffLand_t takeoff_land;

	// ---- control related ----
	Desired_State_t get_hover_des();
	Desired_State_t get_cmd_des();

	// ---- auto takeoff/land ----
	void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
	void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing
	void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
	Desired_State_t get_rotor_speed_up_des(const rclcpp::Time &now);
	Desired_State_t get_takeoff_land_des(const double speed);

	// ---- tools ----
	void set_hov_with_odom();
	void set_hov_with_rc();

	bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
	bool toggle_arm_disarm(bool arm); // It will only try to toggle once, so not blocked.
	void reboot_FCU();

	void publish_bodyrate_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &u, const rclcpp::Time &stamp);
	void publish_trigger(const nav_msgs::msg::Odometry &odom_msg);
};

#endif
