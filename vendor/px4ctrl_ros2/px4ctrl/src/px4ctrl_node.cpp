#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "PX4CtrlFSM.h"

// ROS1原版main()里所有的订阅/发布/服务全部裸写在main()函数体内，直接用
// boost::bind绑到fsm的各个成员上；这里改成一个继承rclcpp::Node的类，
// 在构造函数里做同样的事情（参数加载->构造controller/fsm->挂订阅/发布/
// 服务客户端），本质上是同一套线路图，只是ROS2下更自然的写法是把这些
// 都收进一个Node子类，而不是散在一个裸main()函数里。
//
// 关键设计：FSM构造时只需要一个rclcpp::Node*裸指针（用于
// get_logger()/get_clock()/get_node_base_interface()），不需要Node本身
// 被shared_ptr持有，所以可以在构造函数里用`this`直接构造fsm_，不需要
// 等main()里拿到shared_ptr之后再延迟构造——这点上比"先构造Node、再在
// run()里用shared_from_this()"的写法省事很多。
class Px4CtrlNode : public rclcpp::Node
{
public:
	Px4CtrlNode() : rclcpp::Node("px4ctrl")
	{
		param_.config_from_ros_handle(this);

		controller_ = std::make_unique<LinearControl>(param_);
		fsm_ = std::make_unique<PX4CtrlFSM>(param_, *controller_, this);

		// ---- 订阅 ----
		// 原ROS1版本"odom"/"cmd"是挂在私有命名空间nh("~")下的相对话题名，
		// 靠launch文件里的<remap from="~odom" to="..."/>决定实际话题；
		// ROS2里普通相对名(不带~/前缀)是相对"节点所在namespace"解析，效果
		// 上跟launch里用remappings=[('odom','...')]是等价的，配套的ROS2
		// launch文件(run_ctrl.launch.py)里就是这么remap的。
		// "/mavros/xxx"这些保留原版的绝对路径写法，这是跟上游行为保持
		// 一致的忠实移植；等后续要接入docker_sim双机(NX01/NX02)namespace
		// 隔离时，这批话题需要去掉开头的"/"改成相对名，那是集成阶段的
		// 工作，不在这次移植范围内（README里已经记录了这个待办）。
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

		// !!! 实测炸机复盘（2026-08-07 docker_sim集成后第一次真实起飞测试）!!!
		// 原来这里跟ROS1版本一样用裸整数100当QoS depth，默认解析成
		// Reliable+Volatile。实测`ros2 topic info /NX01/mavros/imu/data
		// --verbose`确认mavros2的imu插件发布用的是Best Effort——DDS里
		// Reliable订阅收不到Best Effort发布者的数据（QoS不兼容，only a
		// WARNING日志"offering incompatible QoS. No messages will be sent
		// to it"，不是错误，非常容易被忽略）。后果：imu_data.q从节点启动
		// 到崩溃全程没被赋值过一次，还是Eigen::Quaterniond默认构造的
		// 未初始化内存（不是零、不是单位四元数，是垃圾值）——
		// controller.cpp里`u.q = imu.q * odom.q.inverse() * q`把这坨垃圾
		// 值乘进了每一帧实际发给PX4的姿态指令。实测attitude_thrust_debug.log
		// 全程可见`target_tilt_from_level=180.0deg`（完全倒扣的目标姿态）、
		// 飞机实际姿态被打到yaw≈142°接近翻转，一起飞就朝正前方（墙的方向）
		// 猛冲后摔机。
		// 改成rclcpp::SensorDataQoS()（Best Effort+depth 5的传感器数据
		// 标准QoS）跟mavros2的发布端对齐，不是拍脑袋换的——已经用
		// `ros2 topic info --verbose`实测确认mavros2的imu/data发布端
		// QoS，这里是照配的，不是猜的。
		imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
			"/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
			rclcpp::SensorDataQoS(),
			[this](const sensor_msgs::msg::Imu::ConstSharedPtr msg)
			{ fsm_->imu_data.feed(msg); });

		// 实测确认（同一次事故排查顺带查的）：mavros2的rc/in插件发布用的是
		// Reliable（不是Best Effort），跟imu/battery不一样，原来的裸整数
		// depth（默认Reliable）本来就是兼容的，不用改。docker_sim这边
		// no_RC恒为true，这段代码实际上从没跑过，但保持原样是为了这个
		// px4ctrl_ros2通用移植脱离docker_sim单独实机使用时行为正确。
		if (!param_.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
		{
			rc_sub_ = create_subscription<mavros_msgs::msg::RCIn>(
				"/mavros/rc/in", 10,
				[this](const mavros_msgs::msg::RCIn::ConstSharedPtr msg)
				{ fsm_->rc_data.feed(msg); });
		}

		// mavros2的battery插件同样是Best Effort发布（实测同一次事故的日志里
		// 能看到跟imu/data一模一样的"offering incompatible QoS"警告），
		// 一并改掉，即使battery本身不直接参与控制律、不是这次事故的主因。
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
		fsm_->debug_pub = create_publisher<quadrotor_msgs::msg::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

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

	// 原ROS1版本这里手写了mySigintHandler+signal(SIGINT,...)去调用
	// ros::shutdown()。rclcpp::init()默认已经装好了自己的SIGINT处理
	// （Ctrl+C会让rclcpp::ok()变false），下面这个手写的主循环本来就是
	// 靠rclcpp::ok()判断退出条件，不需要再额外写一个信号处理函数。
	auto node = std::make_shared<Px4CtrlNode>();

	rclcpp::sleep_for(std::chrono::milliseconds(1000));

	if (node->param().takeoff_land.no_RC)
	{
		RCLCPP_WARN(node->get_logger(), "PX4CTRL] Remote controller disabled, be careful!");
	}
	else
	{
		RCLCPP_INFO(node->get_logger(), "PX4CTRL] Waiting for RC");
		while (rclcpp::ok())
		{
			rclcpp::spin_some(node);
			if (node->fsm().rc_is_received(node->get_clock()->now()))
			{
				RCLCPP_INFO(node->get_logger(), "[PX4CTRL] RC received.");
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

	// 注意：rclcpp::Rate(GenericRate<std::chrono::system_clock>)是按系统
	// 墙钟计时的，不会跟随ROS /clock(use_sim_time)走——跟ROS1的ros::Rate
	// 在use_sim_time=true时会跟随仿真时钟不同。控制主循环用墙钟计时对
	// 100Hz控制律本身没问题(这本来就该按真实物理时间跑)，但如果这套节点
	// 之后被接入Gazebo仿真且RTF(real time factor)明显偏离1，"墙钟100Hz"
	// 跟"仿真时钟100Hz"会不一致，是需要在docker_sim集成阶段留意的一点。
	rclcpp::Rate r(node->param().ctrl_freq_max);
	while (rclcpp::ok())
	{
		r.sleep();
		rclcpp::spin_some(node);
		node->fsm().process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
	}

	rclcpp::shutdown();
	return 0;
}
