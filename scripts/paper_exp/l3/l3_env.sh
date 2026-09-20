#!/usr/bin/env bash
# 容器内跑任何 ROS Python 工具前先 source 它，把 ROS + DDS 环境补齐。
#
# 为什么需要：`docker exec` 进来的是一个全新的 shell，不带 entrypoint 给节点
# 进程导出的那几个变量。不 source ROS 就 `ModuleNotFoundError: No module
# named 'rclpy'`；只 source ROS 不设 DDS 就只能看到 /rosout 和 /parameter_events。
#
# 用法：
#   docker exec -it <容器> bash -c 'source /logs/l3_env.sh; python3 /logs/l3_sep_monitor.py'
#   docker exec -it <容器> bash -c 'source /logs/l3_env.sh; python3 /logs/l3_analyze.py timing /logs/l3/h3_run1 --tag jetson'
source /opt/ros/humble/setup.bash
source /opt/px4ctrl_ws/install/setup.bash 2>/dev/null || true
for pat in origin_setter dlio_odom_node mavros_node point_lio; do
    PID=$(pgrep -f "${pat}" | head -1)
    [ -z "${PID}" ] && continue
    [ -r "/proc/${PID}/environ" ] || continue
    eval "$(tr '\0' '\n' < /proc/${PID}/environ | grep -E '^(RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_DOMAIN_ID)=' | sed 's/^/export /')"
    break
done
echo "== ROS+DDS 就绪: RMW=${RMW_IMPLEMENTATION:-未设置} DOMAIN=${ROS_DOMAIN_ID:-未设置} ==" >&2
