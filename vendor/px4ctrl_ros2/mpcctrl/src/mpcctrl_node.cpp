// mpcctrl_node.cpp——第二版，替换掉第一版那个没有状态机、只有裸MPC控制
// 回路的实现（第一版做过的悬停求解验证/方向一致性验证/新鲜度外推实验都
// 记在DEBUG_JOURNAL.md 2026-09-13"改造rpg_mpc"相关的几条记录里，结论仍然
// 有效——这一版没有改MPC核心数学，只是把裸控制回路接进了完整的px4ctrl FSM）。
//
// 用户要求"补上完整的FSM"：不是重新发明一套状态机，而是照抄so3ctrl已经
// 验证过的模式——PX4CtrlFSM.h/.cpp、input.h/.cpp、PX4CtrlParam.h/.cpp
// 逐字复制自px4ctrl(仅日志字面量[px4ctrl]->[mpcctrl])，只有controller.h/
// .cpp换成调用mpcctrl::MpcController的实现。本文件对应原版px4ctrl_node.cpp/
// so3ctrl_node.cpp，订阅/发布/服务/主循环结构逐字照搬，只改了节点名/
// 调试话题名，以及LinearControl的构造多传一个this(node指针)用来加载MPC
// 自己的代价矩阵/限幅参数。
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "mpcctrl/PX4CtrlFSM.h"

class MpcctrlNode : public rclcpp::Node
{
public:
	MpcctrlNode() : rclcpp::Node("mpcctrl")
	{
		param_.config_from_ros_handle(this);

		controller_ = std::make_unique<LinearControl>(param_, this);
		fsm_ = std::make_unique<PX4CtrlFSM>(param_, *controller_, this);

		// ---- 订阅 ----
		state_sub_ = create_subscription<mavros_msgs::msg::State>(
			"/mavros/state", 10,
			[this](const mavros_msgs::msg::State::ConstSharedPtr msg)
			{ fsm_->state_data.feed(msg); });

		extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
			"/mavros/extended_state", 10,
			[this](const mavros_msgs::msg::ExtendedState::ConstSharedPtr msg)
			{ fsm_->extended_state_data.feed(msg); });

		odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
			"odom", 100,
			[this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
			{ fsm_->odom_data.feed(msg); });

		cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
			"cmd", 100,
			[this](const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg)
			{ fsm_->cmd_data.feed(msg); });

		// IMU/battery固定用SensorDataQoS()跟mavros2的Best Effort发布对齐，
		// 跟px4ctrl_node.cpp同一个理由（见该文件2026-08-07事故复盘注释）。
		imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
			"/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
			rclcpp::SensorDataQoS(),
			[this](const sensor_msgs::msg::Imu::ConstSharedPtr msg)
			{ fsm_->imu_data.feed(msg); });

		if (!param_.takeoff_land.no_RC)
		{
			rc_sub_ = create_subscription<mavros_msgs::msg::RCIn>(
				"/mavros/rc/in", 10,
				[this](const mavros_msgs::msg::RCIn::ConstSharedPtr msg)
				{ fsm_->rc_data.feed(msg); });
		}

		bat_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
			"/mavros/battery", rclcpp::SensorDataQoS(),
			[this](const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
			{ fsm_->bat_data.feed(msg); });

		takeoff_land_sub_ = create_subscription<quadrotor_msgs::msg::TakeoffLand>(
			"takeoff_land", 100,
			[this](const quadrotor_msgs::msg::TakeoffLand::ConstSharedPtr msg)
			{ fsm_->takeoff_land_data.feed(msg); });

		// ---- 发布 ----
		fsm_->ctrl_FCU_pub = create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
		fsm_->traj_start_trigger_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/traj_start_trigger", 10);
		fsm_->debug_pub = create_publisher<quadrotor_msgs::msg::Px4ctrlDebug>("/debugMpcctrl", 10); // debug

		// ---- 服务客户端 ----
		fsm_->set_FCU_mode_srv = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
		fsm_->arming_client_srv = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
		fsm_->reboot_FCU_srv = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
	}

	PX4CtrlFSM &fsm() { return *fsm_; }
	const Parameter_t &param() const { return param_; }

private:
	Parameter_t param_;
	std::unique_ptr<LinearControl> controller_;
	std::unique_ptr<PX4CtrlFSM> fsm_;

	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
	rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
	rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
	rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bat_sub_;
	rclcpp::Subscription<quadrotor_msgs::msg::TakeoffLand>::SharedPtr takeoff_land_sub_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<MpcctrlNode>();

	rclcpp::sleep_for(std::chrono::milliseconds(1000));

	if (node->param().takeoff_land.no_RC)
	{
		RCLCPP_WARN(node->get_logger(), "[mpcctrl] Remote controller disabled, be careful!");
	}
	else
	{
		RCLCPP_INFO(node->get_logger(), "[mpcctrl] Waiting for RC");
		while (rclcpp::ok())
		{
			rclcpp::spin_some(node);
			if (node->fsm().rc_is_received(node->get_clock()->now()))
			{
				RCLCPP_INFO(node->get_logger(), "[mpcctrl] RC received.");
				break;
			}
			rclcpp::sleep_for(std::chrono::milliseconds(100));
		}
	}

	int trials = 0;
	while (rclcpp::ok() && !node->fsm().state_data.current_state.connected)
	{
		rclcpp::spin_some(node);
		rclcpp::sleep_for(std::chrono::milliseconds(1000));
		if (trials++ > 5)
			RCLCPP_ERROR(node->get_logger(), "[mpcctrl] Unable to connnect to PX4!!!");
	}

	rclcpp::Rate r(node->param().ctrl_freq_max);
	while (rclcpp::ok())
	{
		r.sleep();
		rclcpp::spin_some(node);
		node->fsm().process();
	}

	rclcpp::shutdown();
	return 0;
}
