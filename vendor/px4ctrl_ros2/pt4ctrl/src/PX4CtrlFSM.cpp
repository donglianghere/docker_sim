#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

// 少了LinearControl&参数——没有控制律对象需要持有/初始化。
PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, rclcpp::Node *node) : param(param_), node_(node)
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
}

/*
	Finite State Machine（跟px4ctrl逐字相同的状态图——pt4ctrl没有改动
	任何状态转移条件/时序，只是"进入某状态后怎么把参考量变成发给PX4的
	实际指令"这一步换了实现方式，见process()里STEP1之后的注释）

	      system start
	            |
	            |
	            v
	----- > MANUAL_CTRL <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > AUTO_TAKEOFF  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       AUTO_HOVER <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > AUTO_LAND -------
	|         |   |
	|         |   v
	-------- CMD_CTRL

*/

void PX4CtrlFSM::process()
{

	rclcpp::Time now_time = node_->get_clock()->now();
	Desired_State_t des(odom_data);

	// STEP1: state machine runs（跟px4ctrl逐字相同，未作任何改动——反复
	// 起降/RC失控保护/起飞前置检查这些能力全部原样保留，pt4ctrl相对
	// ros2_px4_stack最大的优势就在这一整段代码，一个字都没有改）
	switch (state)
	{
	case MANUAL_CTRL:
	{
		if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER
		{
			if (!odom_is_received(now_time))
			{
				RCLCPP_ERROR(node_->get_logger(), "[pt4ctrl] Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				RCLCPP_ERROR(node_->get_logger(), "[pt4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 3.0)
			{
				RCLCPP_ERROR(node_->get_logger(), "[pt4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}

			state = AUTO_HOVER;
			set_hov_with_odom();
			toggle_offboard_mode(true);

			RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
		}
		else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
		{
			if (!odom_is_received(now_time))
			{
				RCLCPP_ERROR(node_->get_logger(), "[pt4ctrl] Reject AUTO_TAKEOFF. No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				RCLCPP_ERROR(node_->get_logger(), "[pt4ctrl] Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 0.1)
			{
				RCLCPP_ERROR(node_->get_logger(), "[pt4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
				break;
			}
			if (!get_landed())
			{
				RCLCPP_ERROR(node_->get_logger(), "[pt4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
				break;
			}
			if (rc_is_received(now_time)) // Check this only if RC is connected.
			{
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
				{
					RCLCPP_ERROR(node_->get_logger(), "[pt4ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
					while (rclcpp::ok())
					{
						rclcpp::sleep_for(std::chrono::milliseconds(10));
						rclcpp::spin_some(node_->get_node_base_interface());
						if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
						{
							RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] OK, you can takeoff again.\033[32m");
							break;
						}
					}
					break;
				}
			}

			state = AUTO_TAKEOFF;
			set_start_pose_for_takeoff_land(odom_data);
			toggle_offboard_mode(true);				  // toggle on offboard before arm
			for (int i = 0; i < 10 && rclcpp::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU // mark
			{
				rclcpp::sleep_for(std::chrono::milliseconds(10));
				rclcpp::spin_some(node_->get_node_base_interface());
			}
			if (param.takeoff_land.enable_auto_arm)
			{
				toggle_arm_disarm(true);
			}
			takeoff_land.toggle_takeoff_land_time = now_time;

			RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
		}

		if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
		{
			if (state_data.current_state.armed)
			{
				RCLCPP_ERROR(node_->get_logger(), "[pt4ctrl] Reject reboot! Disarm the drone first!");
				break;
			}
			reboot_FCU();
		}

		break;
	}

	case AUTO_HOVER:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			RCLCPP_WARN(node_->get_logger(), "[pt4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
		}
		else if (rc_data.is_command_mode && cmd_is_received(now_time))
		{
			if (state_data.current_state.mode == "OFFBOARD")
			{
				state = CMD_CTRL;
				des = get_cmd_des();
				RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
			}
		}
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND)
		{

			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);

			RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
		}
		else
		{
			set_hov_with_rc();
			des = get_hover_des();
			if ((rc_data.enter_command_mode) ||
				(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			{
				takeoff_land.delay_trigger.first = false;
				publish_trigger(odom_data.msg);
				RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] TRIGGER sent, allow user command.\033[32m");
			}
		}

		break;
	}

	case CMD_CTRL:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			RCLCPP_WARN(node_->get_logger(), "[pt4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			RCLCPP_INFO(node_->get_logger(), "[pt4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		else
		{
			des = get_cmd_des();
		}

		// 2026-08-13新增：px4ctrl原版这里只打ERROR拒绝，要求LAND必须在
		// AUTO_HOVER触发——docker_sim实测发现这条限制在"规划器持续发布
		// 反馈式setpoint流"的场景下(mighty/ego_planner都会一直发cmd，
		// 哪怕已经悬停在目标点不再移动，也不会主动停止发布)会导致
		// cmd_is_received()永远为true，飞机永远等不到那个"没有新指令"的
		// 窗口，CMD_CTRL->AUTO_HOVER这条退路实际上永远走不到，LAND因此
		// 永久被拒绝，降落按钮形同虚设，见DEBUG_JOURNAL.md 2026-08-13
		// "降落按钮好像都不好用"那次排查。改成CMD_CTRL下也能直接触发
		// AUTO_LAND——`state == CMD_CTRL`这个前置条件保证只有在上面
		// RC/odom/cmd都还健康(没有被上面的分支改判成MANUAL_CTRL/
		// AUTO_HOVER)时才允许，跟AUTO_HOVER自己那条LAND分支的安全前提
		// 是同一个级别，不会绕过RC失控保护。降落轨迹本身
		// (get_takeoff_land_des)只在Z方向匀速下降，X/Y钉在触发那一刻的
		// 位置不动——如果触发时飞机还有水平方向速度，会有一次速度突变
		// （从"跟踪轨迹的速度"直接归零转垂直下降），不是逐字等价于"先
		// 悬停稳定再降落"，但比"永远降不下来"更安全，可接受。
		if (state == CMD_CTRL && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::msg::TakeoffLand::LAND)
		{
			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);

			RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] CMD_CTRL(L3) --> AUTO_LAND\033[32m");
		}

		break;
	}

	case AUTO_TAKEOFF:
	{
		// 2026-09-06新增：跟px4ctrl 2026-08-27同一处修复保持一致——原版这里
		// 唯一的退出条件是真的爬升到takeoff_height，没有像AUTO_HOVER/
		// CMD_CTRL/AUTO_LAND那样的"RC悬停开关拨离/odom丢失就退回
		// MANUAL_CTRL"这条保护——真机实测撞上：MANUAL_CTRL->AUTO_TAKEOFF
		// 切换本身成功了(mode切到OFFBOARD)，但紧接着的解锁被PX4拒绝(ARM
		// rejected by PX4!，具体原因是PX4自己的prearm check)。电机没转，
		// 飞机永远爬不到目标高度，状态机因此永久卡死——之后不管拨不拨RC
		// 开关、点不点起飞按钮都没有任何反应，之前只能靠重启进程恢复。
		// 补上跟其它三个飞行状态完全一样的退出检查，给飞手一个不需要
		// 重启进程就能夺回控制权的手段。pt4ctrl此前已经有CMD_CTRL->
		// AUTO_LAND那处修复（是该修复最早的来源），唯独没跟进这一处。
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			RCLCPP_WARN(node_->get_logger(), "[pt4ctrl] AUTO_TAKEOFF --> MANUAL_CTRL(L1)");
		}
		else if ((now_time - takeoff_land.toggle_takeoff_land_time).seconds() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
		{
			des = get_rotor_speed_up_des(now_time);
		}
		else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");

			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second = now_time + rclcpp::Duration::from_seconds(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
		}
		else
		{
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}

		break;
	}

	case AUTO_LAND:
	{
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			RCLCPP_WARN(node_->get_logger(), "[pt4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode)
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			RCLCPP_INFO(node_->get_logger(), "[pt4ctrl] From AUTO_LAND to AUTO_HOVER(L2)!");
		}
		else if (!get_landed())
		{
			des = get_takeoff_land_des(-param.takeoff_land.speed);
		}
		else
		{
			// px4ctrl这里会设rotor_low_speed_during_land=true、STEP3据此
			// 切到motors_idling()发一个固定低推力姿态指令，让桨叶在disarm前
			// "怠速降下来"——pt4ctrl没有姿态指令这个概念，这一整段没有对应
			// 实现。触地后这个分支从没重新赋值过des，仍然是process()开头
			// `Desired_State_t des(odom_data)`那个默认值（位置=当前里程计
			// 位置、速度=0）——效果上已经等价于"发布一个原地悬停的位置
			// setpoint"，跟px4ctrl这里"disarm前保持不动"的目的一致，直接
			// 复用这个默认值，不需要额外处理。这几秒钟内马上就要disarm了，
			// 具体setpoint内容本来影响也很小。

			static bool print_once_flag = true;
			if (print_once_flag)
			{
				RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
				print_once_flag = false;
			}

			if (extended_state_data.current_extended_state.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) // PX4 allows disarm after this
			{
				static double last_trial_time = 0; // Avoid too frequent calls
				if (now_time.seconds() - last_trial_time > 1.0)
				{
					if (toggle_arm_disarm(false)) // disarm
					{
						print_once_flag = true;
						state = MANUAL_CTRL;
						toggle_offboard_mode(false); // toggle off offboard after disarm
						RCLCPP_INFO(node_->get_logger(), "\033[32m[pt4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
					}

					last_trial_time = now_time.seconds();
				}
			}
		}

		break;
	}

	default:
		break;
	}

	// px4ctrl的STEP2(推力模型估计)+STEP3(controller.calculateControl()算
	// 姿态/推力、或者rotor_low_speed_during_land时motors_idling())整个删掉
	// ——pt4ctrl没有控制律，des本身就是要发给PX4的全部内容。

	// STEP4（对应px4ctrl的STEP4）: 把des打包成轨迹setpoint发给PX4，交给
	// 固件自己的位置控制环解算姿态/推力，仿ros2_px4_stack的point_to_traj/
	// _pack_into_traj机制。
	publish_trajectory_setpoint(des, now_time);

	// STEP5: Detect if the drone has landed（跟px4ctrl逐字相同，只读
	// des.p/odom.v，不碰控制律输出，未作任何改动）
	land_detector(state, des, odom_data);

	// STEP6: Clear flags beyound their lifetime（跟px4ctrl逐字相同）
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;
}

void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
	static State_t last_state = State_t::MANUAL_CTRL;
	if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
	{
		takeoff_land.landed = false; // Always holds
	}
	last_state = state;

	if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
	{
		takeoff_land.landed = true;
		return; // No need of other decisions
	}

	// land_detector parameters
	constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
	constexpr double VELOCITY_THR_C = 0.1;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
	constexpr double TIME_KEEP_C = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

	static rclcpp::Time time_C12_reached(0, 0, RCL_ROS_TIME); // time_Constraints12_reached
	static bool is_last_C12_satisfy;
	if (takeoff_land.landed)
	{
		time_C12_reached = node_->get_clock()->now();
		is_last_C12_satisfy = false;
	}
	else
	{
		bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = node_->get_clock()->now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((node_->get_clock()->now() - time_C12_reached).seconds() > TIME_KEEP_C) //Constraint 3 reached
			{
				takeoff_land.landed = true;
			}
		}

		is_last_C12_satisfy = C12_satisfy;
	}
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.a = cmd_data.a;
	des.j = cmd_data.j;
	// 2026-09-07新增yaw锁定开关（param.yaw_lock_enabled，静态参数，
	// PX4CTRL_YAW_LOCK_ENABLED环境变量控制，见docker_sim/DEBUG_JOURNAL.md
	// 同日期条目完整设计讨论）：开启后CMD_CTRL态不再跟随规划器
	// (ego_planner的calculate_yaw())算出来的行进方向朝向，改成锁定在
	// takeoff_land.start_pose(3)——起飞/解锁瞬间用get_yaw_from_quaternion
	// (odom_data.q)已经安全捕获好的实际朝向（set_start_pose_for_takeoff_
	// land()写入），复用这个现成值，不需要额外状态、也不会有"开关触发那
	// 一刻该锁定成多少"的瞬态跳变风险。pt4ctrl不算姿态、把yaw原样转发给
	// PX4自己的位置控制环，锁定朝向同样不影响位置/轨迹跟踪，只是前视
	// 相机不再跟随飞行方向。
	if (param.yaw_lock_enabled)
	{
		des.yaw = takeoff_land.start_pose(3);
		des.yaw_rate = 0.0;
	}
	else
	{
		des.yaw = cmd_data.yaw;
		des.yaw_rate = cmd_data.yaw_rate;
	}

	return des;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const rclcpp::Time &now)
{
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).seconds();
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
	if (des_a_z > 0.1)
	{
		RCLCPP_ERROR(node_->get_logger(), "des_a_z > 0.1!, des_a_z=%f", des_a_z);
		des_a_z = 0.0;
	}

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d(0, 0, des_a_z);
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
	rclcpp::Time now = node_->get_clock()->now();
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).seconds() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
	des.v = Eigen::Vector3d(0, 0, speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = node_->get_clock()->now();
}

void PX4CtrlFSM::set_hov_with_rc()
{
	rclcpp::Time now = node_->get_clock()->now();
	double delta_t = (now - last_set_hover_pose_time).seconds();
	last_set_hover_pose_time = now;

	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land.start_pose.head<3>() = odom_data.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

	takeoff_land.toggle_takeoff_land_time = node_->get_clock()->now();
}

bool PX4CtrlFSM::rc_is_received(const rclcpp::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).seconds() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const rclcpp::Time &now_time)
{
	return (now_time - cmd_data.rcv_stamp).seconds() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const rclcpp::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).seconds() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

// 对应px4ctrl的publish_bodyrate_ctrl()/publish_attitude_ctrl()——那两个
// 方法把Controller_Output_t(姿态四元数+推力)打包成mavros_msgs/AttitudeTarget
// 发到mavros/setpoint_raw/attitude；这个方法把Desired_State_t(位置/速度/
// 加速度/偏航)打包成trajectory_msgs/MultiDOFJointTrajectory发到
// mavros/setpoint_trajectory/local，PX4收到之后自己的位置控制环会解算出
// 姿态/推力，不需要companion computer算这一步。字段映射仿照
// ros2_px4_stack(dynus_offboard_node.py)的point_to_traj/_pack_into_traj：
// transform.translation=位置，velocities.linear=速度，
// accelerations.linear=加速度（前馈项，PX4会拿去做加速度前馈，不是必需
// 但有帮助）。yaw只取偏航角本身（uav_utils::yaw_to_quaternion），不像
// ros2_px4_stack那样用微分平坦算完整的倾角四元数——四旋翼的姿态倾角是
// PX4位置控制环的内部产物，不应该由这一层指定，指定了也没有意义（mavros
// 的setpoint_trajectory插件转成PX4的TRAJECTORY_SETPOINT MAVLink消息时
// 本来就只取yaw分量，四元数里的roll/pitch会被丢弃）。
void PX4CtrlFSM::publish_trajectory_setpoint(const Desired_State_t &des, const rclcpp::Time &stamp)
{
	trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;

	geometry_msgs::msg::Transform transform;
	transform.translation.x = des.p.x();
	transform.translation.y = des.p.y();
	transform.translation.z = des.p.z();
	Eigen::Quaterniond q_yaw = uav_utils::yaw_to_quaternion(des.yaw);
	transform.rotation.x = q_yaw.x();
	transform.rotation.y = q_yaw.y();
	transform.rotation.z = q_yaw.z();
	transform.rotation.w = q_yaw.w();
	point.transforms.push_back(transform);

	geometry_msgs::msg::Twist velocity;
	velocity.linear.x = des.v.x();
	velocity.linear.y = des.v.y();
	velocity.linear.z = des.v.z();
	velocity.angular.z = des.yaw_rate;
	point.velocities.push_back(velocity);

	geometry_msgs::msg::Twist acceleration;
	acceleration.linear.x = des.a.x();
	acceleration.linear.y = des.a.y();
	acceleration.linear.z = des.a.z();
	point.accelerations.push_back(acceleration);

	trajectory_msgs::msg::MultiDOFJointTrajectory msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "map"; // 跟ros2_px4_stack的_pack_into_traj一致
	msg.points.push_back(point);

	traj_setpoint_pub->publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::msg::Odometry &odom_msg)
{
	geometry_msgs::msg::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub->publish(msg);
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
	auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();

	if (on_off)
	{
		state_data.state_before_offboard = state_data.current_state;
		if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
			state_data.state_before_offboard.mode = "MANUAL";

		request->custom_mode = "OFFBOARD";
	}
	else
	{
		request->custom_mode = state_data.state_before_offboard.mode;
	}

	// 跟px4ctrl同样的理由用同步等待写法，process()同样是从px4ctrl_node.cpp
	// 风格的手写主循环调用，不在executor回调栈内部，见px4ctrl这个同名函数
	// 的完整注释。
	if (!set_FCU_mode_srv->wait_for_service(std::chrono::seconds(1)))
	{
		RCLCPP_ERROR(node_->get_logger(), "/mavros/set_mode service not available!");
		return false;
	}

	auto future = set_FCU_mode_srv->async_send_request(request);
	if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future, std::chrono::seconds(1)) !=
		rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(node_->get_logger(), "%s (service call timed out)", on_off ? "Enter OFFBOARD rejected by PX4!" : "Exit OFFBOARD rejected by PX4!");
		return false;
	}

	auto response = future.get();
	if (!response->mode_sent)
	{
		RCLCPP_ERROR(node_->get_logger(), "%s", on_off ? "Enter OFFBOARD rejected by PX4!" : "Exit OFFBOARD rejected by PX4!");
		return false;
	}

	return true;
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
	auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
	request->value = arm;

	if (!arming_client_srv->wait_for_service(std::chrono::seconds(1)))
	{
		RCLCPP_ERROR(node_->get_logger(), "%s (service not available)", arm ? "ARM rejected by PX4!" : "DISARM rejected by PX4!");
		return false;
	}

	auto future = arming_client_srv->async_send_request(request);
	if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future, std::chrono::seconds(1)) !=
		rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(node_->get_logger(), "%s (service call timed out)", arm ? "ARM rejected by PX4!" : "DISARM rejected by PX4!");
		return false;
	}

	auto response = future.get();
	if (!response->success)
	{
		RCLCPP_ERROR(node_->get_logger(), "%s", arm ? "ARM rejected by PX4!" : "DISARM rejected by PX4!");
		return false;
	}

	return true;
}

void PX4CtrlFSM::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
	request->broadcast = false;
	request->command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	request->param1 = 1;	  // Reboot autopilot
	request->param2 = 0;	  // Do nothing for onboard computer
	request->confirmation = true;

	if (!reboot_FCU_srv->wait_for_service(std::chrono::seconds(1)))
	{
		RCLCPP_ERROR(node_->get_logger(), "/mavros/cmd/command service not available!");
		return;
	}

	auto future = reboot_FCU_srv->async_send_request(request);
	rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future, std::chrono::seconds(1));

	RCLCPP_INFO(node_->get_logger(), "Reboot FCU");
}
