// 从px4ctrl的PX4CtrlFSM.h改造而来（不是逐字复制，这点跟so3ctrl不一样——
// so3ctrl只换了controller的内部实现，状态机本身一字未改；pt4ctrl砍掉了
// "des(位置/速度/加速度/偏航参考量) -> u(姿态/推力)"这一整层控制律翻译，
// 状态机STEP1（switch(state){...}产出des）和STEP5（land_detector，只读
// des.p/odom.v）逐字未改，改动只集中在：
//   1. 不再依赖controller.h——Desired_State_t挪到这个文件里定义，
//      Controller_Output_t整个不需要了。
//   2. 构造函数少一个LinearControl&参数。
//   3. motors_idling()/publish_bodyrate_ctrl()/publish_attitude_ctrl()
//      三个跟"姿态指令"绑定的方法全部删掉，换成一个
//      publish_trajectory_setpoint()——直接把des打包成
//      trajectory_msgs/MultiDOFJointTrajectory发布，交给PX4固件自己的
//      位置控制环解算姿态/推力（仿照ros2_px4_stack的point_to_traj/
//      _pack_into_traj机制）。
//   4. debug_pub/Px4ctrlDebug删掉——没有控制律输出可调试，这份debug消息
//      本来就是controller.calculateControl()的返回值。
// 具体每处改动的理由见PX4CtrlFSM.cpp。
#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <rclcpp/rclcpp.hpp>
#include <cassert>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include "input.h"
#include <Eigen/Dense>

// 原来定义在controller.h里，pt4ctrl没有controller.h，搬到这里——字段跟
// px4ctrl的Desired_State_t逐字相同，仍然是"位置/速度/加速度/急动度/姿态/
// 偏航/偏航角速度"这套参考量，只是消费方从"控制律"变成了"轨迹setpoint
// 打包"，接口没有任何变化。
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

struct AutoTakeoffLand_t
{
	bool landed{true};
	// 默认成员初始化必须显式指定RCL_ROS_TIME clock_type，理由见px4ctrl
	// 同名struct的注释——这一段跟px4ctrl逐字相同。
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
	Imu_Data_t imu_data;   // pt4ctrl_node.cpp不订阅imu话题，这个成员永远是
	                       // 默认构造值——保留声明只是为了跟px4ctrl/so3ctrl
	                       // 结构对齐，没有任何代码读取它。
	Command_Data_t cmd_data;
	Battery_Data_t bat_data; // 同上，不订阅，永远是默认值。
	Takeoff_Land_Data_t takeoff_land_data;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traj_start_trigger_pub;
	// 跟px4ctrl的ctrl_FCU_pub（AttitudeTarget，发到mavros/setpoint_raw/attitude）
	// 是对应关系，但pt4ctrl发的是轨迹setpoint，类型和目标话题都不同。
	rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_setpoint_pub;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_FCU_mode_srv;
	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_srv;
	rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr reboot_FCU_srv;

	Eigen::Vector4d hover_pose;
	rclcpp::Time last_set_hover_pose_time{0, 0, RCL_ROS_TIME};

	enum State_t
	{
		MANUAL_CTRL = 1, // pt4ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER, // pt4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// pt4ctrl is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND
	};

	// 少了px4ctrl构造函数里的LinearControl&参数——没有控制律对象需要持有。
	PX4CtrlFSM(Parameter_t &, rclcpp::Node *node);
	void process();
	bool rc_is_received(const rclcpp::Time &now_time);
	bool cmd_is_received(const rclcpp::Time &now_time);
	bool odom_is_received(const rclcpp::Time &now_time);
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

	// px4ctrl这里是publish_bodyrate_ctrl/publish_attitude_ctrl两个方法
	// （按use_bodyrate_ctrl参数二选一），pt4ctrl只有一种发布方式，合并成一个。
	void publish_trajectory_setpoint(const Desired_State_t &des, const rclcpp::Time &stamp);
	void publish_trigger(const nav_msgs::msg::Odometry &odom_msg);
};

#endif
