#include "PX4CtrlParam.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(rclcpp::Node *node)
{
	// 原ROS1参数名用"/"分层（如"gain/Kp0"），ROS2参数名约定用"."分层，
	// 对应到yaml文件里就是ros__parameters下的嵌套map（gain: {Kp0: 1.5, ...}），
	// 加载时会被自动拍平成"gain.Kp0"这样的点分名字。
	read_essential_param(node, "gain.Kp0", gain.Kp0);
	read_essential_param(node, "gain.Kp1", gain.Kp1);
	read_essential_param(node, "gain.Kp2", gain.Kp2);
	read_essential_param(node, "gain.Kv0", gain.Kv0);
	read_essential_param(node, "gain.Kv1", gain.Kv1);
	read_essential_param(node, "gain.Kv2", gain.Kv2);
	read_essential_param(node, "gain.Kvi0", gain.Kvi0);
	read_essential_param(node, "gain.Kvi1", gain.Kvi1);
	read_essential_param(node, "gain.Kvi2", gain.Kvi2);
	read_essential_param(node, "gain.KAngR", gain.KAngR);
	read_essential_param(node, "gain.KAngP", gain.KAngP);
	read_essential_param(node, "gain.KAngY", gain.KAngY);
	// 注意：Kvd0/1/2这三个字段在原ROS1版本里也从来没有被读取/赋值过
	// （struct里声明了，yaml里也有Kvd0/1/2，但config_from_ros_handle
	// 从未读取它们），是上游代码本身的遗留状况，这里原样保留、不额外修复，
	// 保持和ROS1版本功能对等（controller.cpp也从未使用过这三个字段）。

	read_essential_param(node, "rotor_drag.x", rt_drag.x);
	read_essential_param(node, "rotor_drag.y", rt_drag.y);
	read_essential_param(node, "rotor_drag.z", rt_drag.z);
	read_essential_param(node, "rotor_drag.k_thrust_horz", rt_drag.k_thrust_horz);

	read_essential_param(node, "msg_timeout.odom", msg_timeout.odom);
	read_essential_param(node, "msg_timeout.rc", msg_timeout.rc);
	read_essential_param(node, "msg_timeout.cmd", msg_timeout.cmd);
	read_essential_param(node, "msg_timeout.imu", msg_timeout.imu);
	read_essential_param(node, "msg_timeout.bat", msg_timeout.bat);

	read_essential_param(node, "pose_solver", pose_solver);
	read_essential_param(node, "mass", mass);
	read_essential_param(node, "gra", gra);
	read_essential_param(node, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(node, "use_bodyrate_ctrl", use_bodyrate_ctrl);
	read_essential_param(node, "max_manual_vel", max_manual_vel);
	read_essential_param(node, "max_angle", max_angle);
	read_essential_param(node, "low_voltage", low_voltage);
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

	read_essential_param(node, "thrust_model.print_value", thr_map.print_val);
	read_essential_param(node, "thrust_model.K1", thr_map.K1);
	read_essential_param(node, "thrust_model.K2", thr_map.K2);
	read_essential_param(node, "thrust_model.K3", thr_map.K3);
	read_essential_param(node, "thrust_model.accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(node, "thrust_model.hover_percentage", thr_map.hover_percentage);

	max_angle /= (180.0 / M_PI);

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

	if (thr_map.print_val)
	{
		RCLCPP_WARN(node->get_logger(), "You should disable \"print_value\" if you are in regular usage.");
	}
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
