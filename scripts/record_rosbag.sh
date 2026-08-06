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
