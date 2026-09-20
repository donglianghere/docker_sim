// 从px4ctrl的PX4CtrlParam.h搬过来，唯一改动：Gain结构体新增Ki0-2/Kib0-2
// 六个字段——kr_mav_control的SO3Control用到的位置积分增益(世界系+机体系
// 两路)，原版px4ctrl的Gain结构体里没有这两组（Kvi0-2虽然也声明过，但那是
// 原版px4ctrl自己的历史遗留：yaml里有、这里能读到，可controller.cpp从来
// 没真正用它算过东西，不是给SO3Control用的，所以新增字段而不是复用Kvi）。
// 其余内容（RotorDrag/MsgTimeout/ThrustMapping/RCReverse/AutoTakeoffLand
// 等结构体、read_essential_param模板）原样保留，即使其中一些字段
// (Kvd0-2/K1-3/accurate_thrust_model等)在这个控制器里同样用不上——保持
// 跟px4ctrl一致，方便以后需要时对照，不做没必要的删减。
#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <rclcpp/rclcpp.hpp>

class Parameter_t
{
public:
	struct Gain
	{
		double Kp0, Kp1, Kp2;
		double Kv0, Kv1, Kv2;
		double Kvi0, Kvi1, Kvi2;
		double Kvd0, Kvd1, Kvd2;
		double KAngR, KAngP, KAngY;
		// 新增：SO3Control::calculateControl()用到的世界系/机体系位置积分
		// 增益，默认0.0（禁用），需要时手动调大，见config/ctrl_param_fpv.yaml。
		double Ki0, Ki1, Ki2;
		double Kib0, Kib1, Kib2;
	};

	struct RotorDrag
	{
		double x, y, z;
		double k_thrust_horz;
	};

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
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

	Gain gain;
	RotorDrag rt_drag;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;

	int pose_solver;
	double mass;
	double gra;
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;

	bool use_bodyrate_ctrl;
	// bool print_dbg;

	// 2026-09-07新增：yaw锁定开关，静态参数(启动时定死，不支持飞行中
	// 动态切换)，PX4CTRL_YAW_LOCK_ENABLED环境变量控制，默认false(行为
	// 不变)。开启后CMD_CTRL态忽略规划器算出来的行进方向朝向，改成锁定在
	// 起飞瞬间的实际朝向，见PX4CtrlFSM.cpp::get_cmd_des()和
	// docker_sim/DEBUG_JOURNAL.md 2026-09-07相关记录。
	bool yaw_lock_enabled;

	Parameter_t();
	// ROS1原版签名是 config_from_ros_handle(const ros::NodeHandle &nh)，
	// ROS2下参数declare/get都挂在rclcpp::Node本身，改成传Node指针，
	// 函数名保持不变方便和上游对照。
	void config_from_ros_handle(rclcpp::Node *node);
	void config_full_thrust(double hov);

private:
	// ROS1版本用 nh.getParam() 读不到就 ROS_BREAK() 直接abort，语义是
	// "这个参数是必需的，配置文件没给就没法运行"。ROS2下用
	// declare_parameter<TVal>(name)（不带默认值的重载）实现同样的语义：
	// 如果这个参数没有通过外部yaml/命令行覆盖提供初始值，declare_parameter
	// 会抛异常，效果上等价于ROS1的ROS_BREAK()（都是直接让节点起不来），
	// 只是ROS2这边是异常而不是abort。
	//
	// 实测（用flight-stack:latest里的真实Humble核实，不是凭rclcpp文档猜
	// 的，px4ctrl移植时踩过的坑）：Humble下这个重载缺参数时实际抛出的类型是
	// rclcpp::exceptions::UninitializedStaticallyTypedParameterException，
	// 不是文档/直觉上更容易想到的ParameterUninitializedException（这是
	// rclcpp/exceptions/exceptions.hpp里两个完全独立的同级类，都直接继承
	// std::runtime_error，不是父子关系）——一开始按后者写了catch子句，
	// 实测发现完全没catch住，异常一路抛到std::terminate，RCLCPP_FATAL
	// 日志根本没打印出来就直接abort了。这里改成捕获实际会抛出的类型。
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
