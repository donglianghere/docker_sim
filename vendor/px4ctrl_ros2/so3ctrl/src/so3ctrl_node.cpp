// 从px4ctrl的px4ctrl_node.cpp搬过来并做docker_sim多机适配，主要改动：
// 1. 节点名"px4ctrl"→"so3ctrl"。
// 2. 原版px4ctrl_node.cpp里"/mavros/xxx"这批绝对话题名是后来靠
//    patches/px4ctrl_ros2_namespace.patch才改成相对名的（配合
//    namespace=${NAMESPACE} launch，解析成"/NX01/mavros/xxx"）——这里
//    是新写的文件，直接从一开始就用相对名，不需要再走一遍patch。
// 3. imu/battery订阅QoS直接用rclcpp::SensorDataQoS()——这是px4ctrl那边
//    2026-08-07实测炸机复盘换来的教训（mavros2的imu/data发布是Best
//    Effort，订阅端如果用默认Reliable会因QoS不兼容一条都收不到、
//    imu_data.q全程是未初始化垃圾值，直接送出错误姿态指令），这里直接
//    用对的写法，不用再踩一遍。
// 4. controller_/LinearControl这些类型名沿用controller.h/PX4CtrlFSM.h
//    里的历史命名（内部实现已经是SO3几何控制律，类名没改是为了
//    PX4CtrlFSM.h/.cpp能保持逐字复制），这里的成员变量/局部命名同样
//    沿用，不引入"SO3xxx"这种新名字造成不必要的认知负担。
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "PX4CtrlFSM.h"

// 关键设计：FSM构造时只需要一个rclcpp::Node*裸指针（用于
// get_logger()/get_clock()/get_node_base_interface()），不需要Node本身
// 被shared_ptr持有，所以可以在构造函数里用`this`直接构造fsm_，不需要
// 等main()里拿到shared_ptr之后再延迟构造。
class So3CtrlNode : public rclcpp::Node
{
public:
	So3CtrlNode() : rclcpp::Node("so3ctrl")
	{
		param_.config_from_ros_handle(this);

		controller_ = std::make_unique<LinearControl>(param_);
		fsm_ = std::make_unique<PX4CtrlFSM>(param_, *controller_, this);

		// ---- 订阅 ----
		// 全部用相对话题名（不带开头"/"），在节点namespace=NX01下会自动
		// 解析成"/NX01/mavros/xxx"，跟MAVROS的实际namespace
		// （namespace=${NAMESPACE}/mavros，见flight-stack-entrypoint.sh）
		// 对上——这个写法照抄了ros2_px4_stack自己的base_mavros_interface.py
		// 和px4ctrl改过namespace patch之后的做法，是这个项目里板外控制器
		// 统一遵守的既有规范。
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

		// imu/battery用rclcpp::SensorDataQoS()（Best Effort+depth 5的传感器
		// 数据标准QoS），跟mavros2这两个话题的实际发布端QoS对齐——见文件头
		// 注释，这是px4ctrl那边实测炸机换来的教训，这里直接照做。
		imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
			"mavros/imu/data", // Note: do NOT change it to mavros/imu/data_raw !!!
			rclcpp::SensorDataQoS(),
			[this](const sensor_msgs::msg::Imu::ConstSharedPtr msg)
			{ fsm_->imu_data.feed(msg); });

		// mavros2的rc/in插件发布用的是Reliable（不是Best Effort），裸整数
		// depth（默认Reliable）本来就是兼容的，不用改。docker_sim这边
		// no_RC恒为true，这段代码实际上不会跑，保持原样是为了这个so3ctrl
		// 通用移植脱离docker_sim单独实机使用时行为正确。
		if (!param_.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
		{
			rc_sub_ = create_subscription<mavros_msgs::msg::RCIn>(
				"mavros/rc/in", 10,
				[this](const mavros_msgs::msg::RCIn::ConstSharedPtr msg)
				{ fsm_->rc_data.feed(msg); });
		}

		bat_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
			"mavros/battery", rclcpp::SensorDataQoS(),
			[this](const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
			{ fsm_->bat_data.feed(msg); });

		takeoff_land_sub_ = create_subscription<quadrotor_msgs::msg::TakeoffLand>(
			"takeoff_land", 100,
			[this](const quadrotor_msgs::msg::TakeoffLand::ConstSharedPtr msg)
			{ fsm_->takeoff_land_data.feed(msg); });

		// ---- 发布 ----
		fsm_->ctrl_FCU_pub = create_publisher<mavros_msgs::msg::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
		fsm_->traj_start_trigger_pub = create_publisher<geometry_msgs::msg::PoseStamped>("traj_start_trigger", 10);
		fsm_->debug_pub = create_publisher<quadrotor_msgs::msg::Px4ctrlDebug>("debugSo3ctrl", 10); // debug

		// ---- 服务客户端 ----
		fsm_->set_FCU_mode_srv = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
		fsm_->arming_client_srv = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
		fsm_->reboot_FCU_srv = create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");
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

	auto node = std::make_shared<So3CtrlNode>();

	rclcpp::sleep_for(std::chrono::milliseconds(1000));

	if (node->param().takeoff_land.no_RC)
	{
		RCLCPP_WARN(node->get_logger(), "[so3ctrl] Remote controller disabled, be careful!");
	}
	else
	{
		RCLCPP_INFO(node->get_logger(), "[so3ctrl] Waiting for RC");
		while (rclcpp::ok())
		{
			rclcpp::spin_some(node);
			if (node->fsm().rc_is_received(node->get_clock()->now()))
			{
				RCLCPP_INFO(node->get_logger(), "[so3ctrl] RC received.");
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
			RCLCPP_ERROR(node->get_logger(), "[so3ctrl] Unable to connnect to PX4!!!");
	}

	// 注意：rclcpp::Rate是按系统墙钟计时的，不会跟随ROS /clock(use_sim_time)
	// 走——如果Gazebo仿真real-time-factor明显偏离1，"墙钟频率"跟"仿真时钟
	// 频率"会不一致，这是px4ctrl那边已经记录过的已知点，同样适用于这里。
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
