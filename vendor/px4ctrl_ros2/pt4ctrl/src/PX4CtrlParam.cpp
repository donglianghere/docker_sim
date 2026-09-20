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
