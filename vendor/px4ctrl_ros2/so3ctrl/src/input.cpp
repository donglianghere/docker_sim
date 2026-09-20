// 从px4ctrl的input.cpp原样搬过来，跟数据结构/控制律都无关（RC/odom/imu/
// state等原始数据的feed()解析逻辑），唯一改动是把几处静态
// rclcpp::get_logger("px4ctrl")的logger名字改成"so3ctrl"，方便在同一个
// 容器里两边日志混在一起时能用前缀区分是哪个控制器打的（这几行原来就是
// 用固定字符串取logger、不是取节点自己的实际名字，逻辑本身不受影响）。
#include "input.h"

RC_Data_t::RC_Data_t()
{
    rcv_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    last_mode = -1.0;
    last_gear = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    is_hover_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    toggle_reboot = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(mavros_msgs::msg::RCIn::ConstSharedPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = clock_.now();

    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    mode = ((double)msg.channels[4] - 1000.0) / 1000.0;
    gear = ((double)msg.channels[5] - 1000.0) / 1000.0;
    reboot_cmd = ((double)msg.channels[7] - 1000.0) / 1000.0;

    check_validity();

    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }
    if (!have_init_last_reboot_cmd)
    {
        have_init_last_reboot_cmd = true;
        last_reboot_cmd = reboot_cmd;
    }

    // 1
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    else
        enter_hover_mode = false;

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_hover_mode = true;
    else
        is_hover_mode = false;

    // 2
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
            enter_command_mode = true;
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;

        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }

    // 3
    if (!is_hover_mode && !is_command_mode)
    {
        if (last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
            toggle_reboot = true;
        else
            toggle_reboot = false;
    }
    else
        toggle_reboot = false;

    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // pass
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("so3ctrl"), "RC data validity check fail. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
    }
}

bool RC_Data_t::check_centered()
{
    bool centered = abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5;
    return centered;
}

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    q.setIdentity();
    recv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::msg::Odometry::ConstSharedPtr pMsg)
{
    rclcpp::Time now = clock_.now();

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;

    uav_utils::extract_odometry(pMsg, p, v, q, w);

// #define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        RCLCPP_WARN(rclcpp::get_logger("so3ctrl"), "VEL_IN_BODY!!!");
#endif

    // check the frequency
    static int one_min_count = 9999;
    static rclcpp::Time last_clear_count_time(0, 0, RCL_ROS_TIME);
    if ( (now - last_clear_count_time).seconds() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            RCLCPP_WARN(rclcpp::get_logger("so3ctrl"), "ODOM frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    // 防御性初始化：q/w/a是Eigen类型，默认构造不会清零/置单位阵，是未初始化
    // 内存。ROS1原版没有这一段（TCPROS没有"话题静默订阅不到"这种失败模式，
    // 只要发布者存在就一定能收到，这个初始化在ROS1里纯粹是无用功）；ROS2的
    // QoS机制会出现"发布者QoS跟订阅者不兼容->一条都收不到->只有一行WARN
    // 日志，没有error/异常"这种新的静默失败模式（px4ctrl那边2026-08-07实测
    // 就是因为mavros2的imu/data发布是Best Effort、订阅端之前是默认Reliable，
    // 两者不兼容导致imu_data.q从头到尾没被赋值过，垃圾值被算进控制律、直接
    // 送出一个完全错误的姿态指令、起飞就翻了，so3ctrl_node.cpp的imu/battery
    // 订阅从一开始就写成跟mavros2匹配的SensorDataQoS()，见该文件注释），这里
    // 再加一层兜底——万一以后又有哪个话题因为别的原因收不到，至少是"明确的
    // 安全默认值(单位姿态/零角速度/零加速度)"而不是"未定义内存"，二者都是错
    // 的，但后者是纯粹的运气，前者至少是可预期、可复现的错误。
    q.setIdentity();
    w.setZero();
    a.setZero();
}

void Imu_Data_t::feed(sensor_msgs::msg::Imu::ConstSharedPtr pMsg)
{
    rclcpp::Time now = clock_.now();

    msg = *pMsg;
    rcv_stamp = now;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    // check the frequency
    static int one_min_count = 9999;
    static rclcpp::Time last_clear_count_time(0, 0, RCL_ROS_TIME);
    if ( (now - last_clear_count_time).seconds() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            RCLCPP_WARN(rclcpp::get_logger("so3ctrl"), "IMU frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

State_Data_t::State_Data_t()
{
}

void State_Data_t::feed(mavros_msgs::msg::State::ConstSharedPtr pMsg)
{

    current_state = *pMsg;
}

ExtendedState_Data_t::ExtendedState_Data_t()
{
}

void ExtendedState_Data_t::feed(mavros_msgs::msg::ExtendedState::ConstSharedPtr pMsg)
{
    current_extended_state = *pMsg;
}

Command_Data_t::Command_Data_t()
{
    rcv_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void Command_Data_t::feed(quadrotor_msgs::msg::PositionCommand::ConstSharedPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = clock_.now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void Battery_Data_t::feed(sensor_msgs::msg::BatteryState::ConstSharedPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = clock_.now();

    double voltage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        voltage += pMsg->cell_voltage[i];
    }
    volt = 0.8 * volt + 0.2 * voltage; // Naive LPF, cell_voltage has a higher frequency

    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    percentage = pMsg->percentage;

    static rclcpp::Time last_print_t(0, 0, RCL_ROS_TIME);
    if (percentage > 0.05)
    {
        if ((rcv_stamp - last_print_t).seconds() > 10)
        {
            RCLCPP_INFO(rclcpp::get_logger("so3ctrl"), "[so3ctrl] Voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
    else
    {
        if ((rcv_stamp - last_print_t).seconds() > 1)
        {
            // RCLCPP_ERROR(rclcpp::get_logger("so3ctrl"), "[so3ctrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::msg::TakeoffLand::ConstSharedPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = clock_.now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
}
