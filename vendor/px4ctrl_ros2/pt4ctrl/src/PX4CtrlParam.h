// 从px4ctrl的PX4CtrlParam.h精简而来（不是逐字复制）——删掉了Gain/RotorDrag/
// ThrustMapping三个struct，以及mass/gra/pose_solver/max_angle/low_voltage/
// use_bodyrate_ctrl这几个字段：这些全部只在px4ctrl的controller.cpp（控制律）
// 里被读取（已用grep核实过，pt4ctrl没有controller.cpp，删掉这些字段不会
// 留下任何"声明了但config_from_ros_handle读不到值就直接报错退出"的死角，
// 因为PX4CtrlParam.cpp里对应的read_essential_param调用也一并删掉了）。
// 保留的字段（MsgTimeout的odom/rc/cmd、RCReverse、AutoTakeoffLand、
// ctrl_freq_max、max_manual_vel）全部是状态机/RC/主循环实际用到的。
#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <rclcpp/rclcpp.hpp>

class Parameter_t
{
public:
	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};

	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	AutoTakeoffLand takeoff_land;

	double ctrl_freq_max;
	double max_manual_vel;

	// 2026-09-07新增：yaw锁定开关，静态参数(启动时定死，不支持飞行中
	// 动态切换)，PX4CTRL_YAW_LOCK_ENABLED环境变量控制，默认false(行为
	// 不变)。开启后CMD_CTRL态忽略规划器算出来的行进方向朝向，改成锁定在
	// 起飞瞬间的实际朝向，见PX4CtrlFSM.cpp::get_cmd_des()和
	// docker_sim/DEBUG_JOURNAL.md 2026-09-07相关记录。
	bool yaw_lock_enabled;

	Parameter_t();
	void config_from_ros_handle(rclcpp::Node *node);

private:
	// 逐字照抄px4ctrl的read_essential_param实现，见该文件同名函数的完整
	// 注释（Humble下缺参数实际抛出的异常类型、为什么要显式catch这一处）。
	template <typename TVal>
	void read_essential_param(rclcpp::Node *node, const std::string &name, TVal &val)
	{
		try
		{
			val = node->declare_parameter<TVal>(name);
		}
		catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &)
		{
			RCLCPP_FATAL(node->get_logger(), "Read param: %s failed (not provided by parameter file).", name.c_str());
			throw;
		}
	};
};

#endif
