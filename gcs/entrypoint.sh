#!/usr/bin/env bash
# GCS容器entrypoint，2026-08-18新增。
#
# 同时起两个长驻进程：
#   1. rosbridge_websocket（9090端口）——把ROS2话题转成WebSocket JSON，
#      前端靠它拿真机实时参数。
#   2. python3 http.server（8080端口）——serve /opt/frontend下的静态
#      网页，浏览器打开 http://<GCS-IP>:8080 就能用。
# network_mode:host（见docker-compose.yml）下这两个端口就是宿主机端口，
# 不需要额外的-p映射。
#
# rosbridge如果异常退出（比如DDS环境没起来），容器不跟着退出——保留
# "docker exec进去用ros2 CLI排查"这条路，跟这个镜像原来"CMD sleep
# infinity"的长驻定位一致，不做成"一个进程挂了就整个容器重启"的强绑定。
set -eo pipefail

# 2026-08-19验证rviz2链路时实测炸出来的bug：ROS2 setup.bash内部引用了一堆
# 没给默认值的变量（AMENT_TRACE_SETUP_FILES是第一个炸的），跟`set -u`天生
# 冲突，直接把整个容器启动就搞崩了（source这一步报unbound variable直接
# exit，rosbridge/http.server都没起来）。这份entrypoint当初是照着"标准
# 防呆写法"写的`set -uo pipefail`，没对着docker/entrypoints/sim-world-
# entrypoint.sh（同样的坑，已经踩过一次并写了这条注释）抄——source期间关掉
# -u，source完再打开，后面写的变量都用${VAR:-default}，不会再受影响。
set +u
source /opt/ros/humble/setup.bash
source /opt/gcs_ws/install/setup.bash
set -u

# 2026-08-19检查时发现的bug：docker-compose.yml原来把RMW_IMPLEMENTATION
# 硬编码成rmw_cyclonedds_cpp，只对PLANNER=ego_planner成立——flight-stack-
# entrypoint.sh/sim-world-entrypoint.sh都明确写了"只在PLANNER=ego_planner
# 时切换，mighty这条已验证路径继续用默认FastDDS"，且要求"跟PLANNER联动的
# 地方都要同步改，不能只改一处"。GCS这份配置当初写死cyclonedds是基于
# "2026-08-14确认的真机首飞组合是PLANNER=ego_planner"这个假设，没有跟着
# 联动——如果真机切到PLANNER=mighty飞（gcs/rviz/multi_mighty.rviz就是给
# 这个场景准备的），GCS(cyclonedds)跟Jetson(默认FastDDS)会用不同RMW，
# DDS互相发现不了，GCS什么都监控不到。这里补上跟另外两处一致的联动逻辑，
# 必须在下面rosbridge/任何ros2命令启动前设置。
if [ "${PLANNER:-ego_planner}" = "ego_planner" ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    echo "[entrypoint] PLANNER=ego_planner，RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
else
    echo "[entrypoint] PLANNER=${PLANNER}，保持默认RMW（FastDDS），不切换"
fi

echo "[entrypoint] 启动 rosbridge_websocket（端口9090）..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &

echo "[entrypoint] 启动前端静态文件服务（端口8080）..."
python3 -m http.server 8080 --directory /opt/frontend &

# exec原CMD（默认sleep infinity），成为容器PID1，上面两个后台进程作为
# 子进程存活；docker stop发SIGTERM给这个进程，两个后台任务跟着容器一起
# 结束，不需要额外trap。
exec "$@"
