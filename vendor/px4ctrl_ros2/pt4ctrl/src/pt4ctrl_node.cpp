#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "PX4CtrlFSM.h"

// 从px4ctrl_node.cpp改造而来。跟px4ctrl比少了：
//   - controller_(LinearControl)成员，fsm_构造时不再传它
//   - imu_sub_/bat_sub_两个订阅——pt4ctrl没有控制律，imu_data/bat_data
//     这两个FSM成员永远用不到，不订阅省得再操心px4ctrl当年踩过的那个
//     imu话题QoS不匹配的坑（mavros2的imu/data是Best Effort，见px4ctrl_node.cpp
//     那段2026-08-07事故复盘注释——pt4ctrl这条链路上根本不存在会被这个坑
//     影响的代码路径）
//   - debug_pub(Px4ctrlDebug)——没有控制律输出可调试
// 多了：traj_setpoint_pub，发到mavros/setpoint_trajectory/local，取代
// px4ctrl的ctrl_FCU_pub(mavros/setpoint_raw/attitude)。
//
// 话题/服务名直接写成docker_sim集成后的相对路径形式（"mavros/xxx"不带
// 开头"/"）——px4ctrl自己是先以绝对路径"/mavros/xxx"忠实移植上游，再靠
// patches/px4ctrl_ros2_namespace.patch在docker build时改成相对路径，这是
// 因为px4ctrl/so3ctrl是从外部仓库原样搬来的vendor代码，改动要跟"这是我们
// 加的docker_sim定制"这件事分开记录；pt4ctrl是这个repo原生的新文件，没有
// "忠实移植上游"这层顾虑，直接写对，不需要为一个新文件另外维护一份patch。
class Pt4CtrlNode : public rclcpp::Node
{
public:
	Pt4CtrlNode() : rclcpp::Node("pt4ctrl")
	{
		param_.config_from_ros_handle(this);

		fsm_ = std::make_unique<PX4CtrlFSM>(param_, this);

		// ---- 订阅 ----
		state_sub_ = create_subscription<mavros_msgs::msg::State>(
			"mavros/state", 10,
			[this](const mavros_msgs::msg::State::ConstSharedPtr msg)
			{ fsm_->state_data.feed(msg); });

		extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
			"mavros/extended_state", 10,
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

		// mavros2的rc/in插件发布是Reliable（不是Best Effort），裸整数depth
		// 本来就兼容，照抄px4ctrl的写法。docker_sim容器里no_RC恒为true，
		// 这段代码实际没跑过，保留是为了这个包脱离docker_sim单独实机使用
		// 时行为正确（跟px4ctrl_node.cpp的既有说明一致）。
		if (!param_.takeoff_land.no_RC)
		{
			rc_sub_ = create_subscription<mavros_msgs::msg::RCIn>(
				"mavros/rc/in", 10,
				[this](const mavros_msgs::msg::RCIn::ConstSharedPtr msg)
				{ fsm_->rc_data.feed(msg); });
		}

		takeoff_land_sub_ = create_subscription<quadrotor_msgs::msg::TakeoffLand>(
			"takeoff_land", 100,
			[this](const quadrotor_msgs::msg::TakeoffLand::ConstSharedPtr msg)
			{ fsm_->takeoff_land_data.feed(msg); });

		// ---- 发布 ----
		fsm_->traj_setpoint_pub = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
			"mavros/setpoint_trajectory/local", 10);
		fsm_->traj_start_trigger_pub = create_publisher<geometry_msgs::msg::PoseStamped>("traj_start_trigger", 10);

		// ---- 服务客户端 ----
		fsm_->set_FCU_mode_srv = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
		fsm_->arming_client_srv = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
		fsm_->reboot_FCU_srv = create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");
	}

	PX4CtrlFSM &fsm() { return *fsm_; }
	const Parameter_t &param() const { return param_; }

private:
	Parameter_t param_;
	std::unique_ptr<PX4CtrlFSM> fsm_;

	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
	rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
	rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
	rclcpp::Subscription<quadrotor_msgs::msg::TakeoffLand>::SharedPtr takeoff_land_sub_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<Pt4CtrlNode>();

	rclcpp::sleep_for(std::chrono::milliseconds(1000));

	if (node->param().takeoff_land.no_RC)
	{
		RCLCPP_WARN(node->get_logger(), "PT4CTRL] Remote controller disabled, be careful!");
	}
	else
	{
		RCLCPP_INFO(node->get_logger(), "PT4CTRL] Waiting for RC");
		while (rclcpp::ok())
		{
			rclcpp::spin_some(node);
			if (node->fsm().rc_is_received(node->get_clock()->now()))
			{
				RCLCPP_INFO(node->get_logger(), "[PT4CTRL] RC received.");
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
			RCLCPP_ERROR(node->get_logger(), "Unable to connnect to PX4!!!");
	}

	// 跟px4ctrl同样按墙钟计时，同样的use_sim_time/RTF注意事项，见
	// px4ctrl_node.cpp同一处的完整说明。
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
