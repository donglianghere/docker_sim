#!/usr/bin/env bash
# status窗口启动脚本——2026-08-13从watch_sim.sh里内联的一长串`&&`命令链
# 抽出来。原来的写法（docker cp到nx01 && docker cp到nx02 && ... &&
# 最后才ros2 run status_monitor.py）在flight-stack-nx02容器不存在时（单机
# 仿真彩排模式，LOCALIZATION_SOURCE=single_*，见README"单机仿真彩排"
# 一节）会在第二步"docker cp ... nx02"就失败——`&&`串联导致后面所有命令
# （包括真正启动status_monitor.py那一句）全部不会执行，不是"缺NX02那部分
# 数据"，是NX01自己的状态监控也一起看不到。
#
# 这个脚本用`docker compose ps --status running`探测flight-stack-nx02是不是
# 真的在跑，动态决定：①要不要执行NX02相关的docker cp/exec；②传给
# status_monitor.py的命名空间列表是"NX01"还是"NX01 NX02"。status_monitor.py
# 自己本来就支持只传一个命名空间（`len(namespaces) >= 2`才处理UWB双机相关
# 逻辑，见该文件第328行），这边不需要改，只是调用方式要跟着探测结果变。
#
# 用法：由scripts/watch_sim.sh的status窗口调用，也可以手动单独跑
#   ./scripts/status_window.sh
set -eo pipefail
cd "$(dirname "$0")/.."

RUNNING_SERVICES=$(docker compose ps --status running --services 2>/dev/null || true)
NX02_UP=false
if echo "$RUNNING_SERVICES" | grep -qx "flight-stack-nx02"; then
    NX02_UP=true
fi

docker cp scripts/ros2_env_setup.sh docker_sim-flight-stack-nx01-1:/tmp/ros2_env_setup.sh
docker cp scripts/status_monitor.py docker_sim-flight-stack-nx01-1:/tmp/status_monitor.py
docker cp scripts/attitude_thrust_logger.py docker_sim-flight-stack-nx01-1:/tmp/attitude_thrust_logger.py
docker exec -d docker_sim-flight-stack-nx01-1 bash -c \
    'source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && python3 /tmp/attitude_thrust_logger.py NX01 >> /tmp/attitude_thrust_logger_stdout.log 2>&1'

NAMESPACES="NX01"
if [ "$NX02_UP" = true ]; then
    echo "== 检测到 flight-stack-nx02 在跑，status窗口按双机模式启动 =="
    docker cp scripts/ros2_env_setup.sh docker_sim-flight-stack-nx02-1:/tmp/ros2_env_setup.sh
    docker cp scripts/attitude_thrust_logger.py docker_sim-flight-stack-nx02-1:/tmp/attitude_thrust_logger.py
    docker exec -d docker_sim-flight-stack-nx02-1 bash -c \
        'source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && python3 /tmp/attitude_thrust_logger.py NX02 >> /tmp/attitude_thrust_logger_stdout.log 2>&1'
    NAMESPACES="NX01 NX02"
else
    echo "== 未检测到 flight-stack-nx02 在跑，status窗口按单机模式启动（只监控NX01）=="
fi

# collect_container_stats.sh对不存在的容器有2>/dev/null兜底（docker stats
# 跳过它，不导致整条命令失败），不需要按NX02_UP再区分，原样启动即可。
(nohup ./scripts/collect_container_stats.sh >> runtime_logs/collect_container_stats_stdout.log 2>&1 &)

docker exec docker_sim-flight-stack-nx01-1 bash -c \
    "source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && python3 /tmp/status_monitor.py ${NAMESPACES}"
