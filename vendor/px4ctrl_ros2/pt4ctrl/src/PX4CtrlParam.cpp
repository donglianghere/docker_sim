#include <cmath>
#include "PX4CtrlParam.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(rclcpp::Node *node)
{
	read_essential_param(node, "msg_timeout.odom", msg_timeout.odom);
	read_essential_param(node, "msg_timeout.rc", msg_timeout.rc);
	read_essential_param(node, "msg_timeout.cmd", msg_timeout.cmd);

	read_essential_param(node, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(node, "max_manual_vel", max_manual_vel);
	read_essential_param(node, "yaw_lock_enabled", yaw_lock_enabled);

	read_essential_param(node, "rc_reverse.roll", rc_reverse.roll);
	read_essential_param(node, "rc_reverse.pitch", rc_reverse.pitch);
	read_essential_param(node, "rc_reverse.yaw", rc_reverse.yaw);
	read_essential_param(node, "rc_reverse.throttle", rc_reverse.throttle);

	read_essential_param(node, "auto_takeoff_land.enable", takeoff_land.enable);
	read_essential_param(node, "auto_takeoff_land.enable_auto_arm", takeoff_land.enable_auto_arm);
	read_essential_param(node, "auto_takeoff_land.no_RC", takeoff_land.no_RC);
	read_essential_param(node, "auto_takeoff_land.takeoff_height", takeoff_land.height);
	read_essential_param(node, "auto_takeoff_land.takeoff_land_speed", takeoff_land.speed);

	if (takeoff_land.enable_auto_arm && !takeoff_land.enable)
	{
		takeoff_land.enable_auto_arm = false;
		RCLCPP_ERROR(node->get_logger(), "\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	}
	if (takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable))
	{
		takeoff_land.no_RC = false;
		RCLCPP_ERROR(node->get_logger(), "\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}
};

// docker_sim 2026-09-25：见头文件说明。只重读这一个参数——起飞高度是唯一一个
// "任务过程中会想改"的量（每段任务起飞到各自的巡航高度），其余参数保持"启动时
// 读一次"的语义不变，避免运行中被意外改动影响控制律。
void Parameter_t::refresh_takeoff_height(rclcpp::Node *node)
{
	double h = takeoff_land.height;
	node->get_parameter("auto_takeoff_land.takeoff_height", h);
	if (h > 0.0 && std::fabs(h - takeoff_land.height) > 1e-6)
	{
		RCLCPP_INFO(node->get_logger(),
					"\033[32m[pt4ctrl] 起飞高度已更新: %.2f -> %.2f m\033[32m",
					takeoff_land.height, h);
		takeoff_land.height = h;
	}
}
