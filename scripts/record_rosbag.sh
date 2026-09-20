#!/usr/bin/env bash
# "黑匣子"式滚动录制：只录关键、体积可控的话题（TF、双机共享轨迹、
# mavros状态/位姿、目标点、占据栅格、UWB真值），不录原始点云
# （mid360一帧2万点，是这些话题里体积最大的一个数量级以上，默认不录，
# 需要专门查DLIO/点云问题时再手动单独起一次全量录制，不常驻）。
#
# 实测踩过一个坑，专门记录：`occupancy_grid`/`unknown_grid`这两个名字
# 听着像轻量的nav_msgs/OccupancyGrid，实际类型是`sensor_msgs/PointCloud2`
# （global_mapper_ros.cc里就是拿PointCloud2发布的，"grid"只是话题名字
# 沿用的老称呼）——`ros2 topic bw`实测：`occupancy_grid`约36KB/s/机还好，
# 但`unknown_grid`（20x20米房间里"未知"格子远多于"占据"格子，点数多
# 一个数量级）飙到约410~460KB/s/机，双机合计能占掉滚动录制总带宽的
# 七成以上——录了几分钟就能吃掉几百MB，跟"不让存储爆"这个目标直接冲突。
# 所以默认列表里只留`occupancy_grid`、去掉`unknown_grid`；专门要查
# frontier/exploration相关问题时，再手动加回来单独录一次，不常驻。
#
# 用`timeout`每隔MAX_DURATION秒重开一个新bag目录，而不是用
# `ros2 bag record --max-bag-duration`在同一个bag目录里做内部分片——
# 分片后所有分片共用一份metadata.yaml，配套的prune_rosbag.sh要删除
# 旧数据时没法只删部分分片（会把metadata.yaml和实际文件对不上，
# `ros2 bag info`可能读不出来）。每次开一个全新、独立、能整体删除的
# bag目录，配合prune_rosbag.sh"删最老的几个目录"就是安全的。
#
# 用法（在能看到全部双机话题的容器里跑，比如flight-stack-nx01，
# network_mode:host下所有容器的话题互相可见）：
#   ./scripts/record_rosbag.sh
set -eo pipefail
# 只source humble基础环境的话，ros2 bag record认不出/trajs这类自定义消息
# 类型（dynus_interfaces/msg/DynTraj），会整条跳过、只打印警告——要跟
# flight-stack-entrypoint.sh一样，把mighty/dlio/ros2_px4_stack三个工作区
# 的install/setup.bash也叠加source上，才能让这些包的消息类型被正确识别。
source /opt/ros/humble/setup.bash
source /opt/decomp_ws/install/setup.bash 2>/dev/null || true
source /opt/mighty_ws/install/setup.bash
source /opt/dlio_ws/install/setup.bash
source /opt/ros2_px4_stack_ws/install/setup.bash
# 2026-08-08新增录ego_planner这批话题之后补的——traj_utils/msg/Bspline
# （/NX01/planning/bspline）来自ego_planner_ws，quadrotor_msgs/msg/
# PositionCommand（/NX01/position_cmd）和Px4ctrlDebug（/NX01/debugPx4ctrl）
# 来自px4ctrl_ws，不source这两个工作区，跟上面注释里说的/trajs一样的坑：
# ros2 bag record认不出这些自定义消息类型，会跳过、只打印警告。
source /opt/ego_planner_ws/install/setup.bash
source /opt/px4ctrl_ws/install/setup.bash

# 这个脚本是docker exec起的全新shell进程，跟entrypoint.sh自己那个shell
# 完全独立，PLANNER=ego_planner时必须自己重新切RMW才能录到真实节点的
# 消息（不补的话bag只有话题名、没有任何消息）——完整原因、还有"容器刚
# 重启配置文件还没写出来"这个启动时序竞态的说明，都在
# scripts/ros2_env_setup.sh自己的注释里，跟watch_sim.sh共用同一份，
# 不在这里重复。这个脚本是被docker cp进容器再执行的独立文件，运行时跟
# ros2_env_setup.sh在同一个/tmp目录下，直接source相对路径就行。
source "$(dirname "${BASH_SOURCE[0]}")/ros2_env_setup.sh"

OUT_DIR="${OUT_DIR:-/logs/rosbag}"
MAX_DURATION="${MAX_DURATION:-300}"  # 秒，默认5分钟一个块
mkdir -p "$OUT_DIR"

TOPICS=(
  /tf /tf_static
  /trajs
  /NX01/term_goal /NX02/term_goal
  /NX01/mavros/state /NX02/mavros/state
  /NX01/mavros/local_position/pose /NX02/mavros/local_position/pose
  /NX01/occupancy_grid /NX02/occupancy_grid
  /frame_align/NX01/NX02 /frame_align/NX02/NX01
  /plug/model_states_plug
  # 2026-08-08用户要求实时录目标/规划轨迹/实际轨迹/实际输出——下面这批是
  # PLANNER=ego_planner时才有的话题，mighty模式下这些话题不存在，
  # `ros2 bag record`只会一直等、不会报错（不影响上面mighty那批话题正常
  # 录），两边共用同一份列表，不用按PLANNER分支维护两份。
  # 目标：rviz_goal_world是2D Goal Pose点出来的原始世界坐标(offset换算前)
  /NX01/rviz_goal_world /NX02/rviz_goal_world
  # 规划轨迹：optimal_list是B样条优化后的最终轨迹，init_list/global_list
  # 是中间过程（A*粗路径/优化前初值），position_cmd是traj_server按10ms
  # 节拍采样这条轨迹之后真正发给px4ctrl的逐帧指令（含yaw/yaw_dot）
  /NX01/optimal_list /NX02/optimal_list
  /NX01/init_list /NX02/init_list
  /NX01/global_list /NX02/global_list
  /NX01/planning/bspline /NX02/planning/bspline
  /NX01/position_cmd /NX02/position_cmd
  # 实际轨迹：DLIO自己的里程计（比mavros/local_position/pose多了速度/
  # 姿态协方差这些字段），跟position_cmd对齐能看出跟踪误差
  /NX01/dlio/odom_node/odom /NX02/dlio/odom_node/odom
  # 2026-09-10新增：DLIO原始话题（上面那行）不等于真正喂给PX4 EKF2的
  # 频率——DLIO自己的100Hz发布定时器混了IMU传播/真实雷达修正两种帧，
  # PX4侧ulog里estimator_aid_src_ev_pos/ev_hgt又被固件日志系统按固定
  # 500ms/2Hz抽样记录（见logged_topics.cpp的kEKFVerboseIntervalMilliseconds），
  # 事后从ulog反推不出repub_odom真正转发给mavros的频率。这一路是
  # repub_odom.py转发出来、mavros再转给PX4的那一路，录下来才能算出
  # EKF2外部视觉输入的真实到达频率，不用再靠ulog反推。
  /NX01/mavros/vision_pose/pose_cov /NX02/mavros/vision_pose/pose_cov
  # 实际输出：px4ctrl真正发给PX4的姿态/推力目标 vs mavros上报的实际IMU
  # 姿态，加上px4ctrl自己的调试话题
  /NX01/mavros/setpoint_raw/target_attitude /NX02/mavros/setpoint_raw/target_attitude
  /NX01/mavros/imu/data /NX02/mavros/imu/data
  /NX01/debugPx4ctrl /NX02/debugPx4ctrl
  # ego_planner用的占据栅格（跟mighty的occupancy_grid不是同一个话题/类型）
  /NX01/grid_map/occupancy_inflate /NX02/grid_map/occupancy_inflate
  # 2026-09-03新增：SE(2)在线标定论文的离线复算需要的四路数据（见
  # `docker_sim/实验方案_SE2在线标定论文补充实验.md` L1层）。有了这几路，
  # 录一次飞行就能在宿主机上离线把W/d_min/批量/滑窗/Huber全部扫一遍，
  # 不用每换一组参数就重飞一次。
  #   pose_abs   : origin_setter真正吃的那一路带噪声绝对位置观测
  #   pose_truth : 同一时刻的Gazebo真值位姿(无噪声，含真实yaw)——评估用，
  #                特意发成geometry_msgs/PoseStamped而不是让离线工具去解
  #                gazebo_msgs/ModelStates，因为宿主机上装的ROS没有
  #                gazebo_msgs这个包，解不了那个类型
  #   yaw_estimate/yaw_sample_count: 机上估计器当时的实际输出，用来跟
  #                离线复算结果对照，确认离线那份逻辑跟机上一致
  /NX01/uwb/pose_abs /NX02/uwb/pose_abs
  /NX01/uwb/pose_truth /NX02/uwb/pose_truth
  /NX01/origin_setter/yaw_estimate /NX02/origin_setter/yaw_estimate
  /NX01/origin_setter/yaw_sample_count /NX02/origin_setter/yaw_sample_count
)

echo "== record_rosbag: 每${MAX_DURATION}秒一个块，写到 ${OUT_DIR}/bag_<时间戳> =="
while true; do
  ts=$(date +%Y%m%d_%H%M%S)
  bag_dir="${OUT_DIR}/bag_${ts}"
  echo "== 开始新块: ${bag_dir} =="
  # timeout发SIGTERM给ros2 bag record，它会正常收尾（写完metadata.yaml）
  # 再退出，不是强杀——`|| true`是因为timeout超时触发SIGTERM本身会让
  # 这条命令返回非0，不代表录制失败。
  timeout --signal=TERM "$MAX_DURATION" \
    ros2 bag record -o "$bag_dir" "${TOPICS[@]}" || true
done
