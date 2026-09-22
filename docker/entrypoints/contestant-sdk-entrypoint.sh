#!/usr/bin/env bash
# contestant-sdk 容器入口：选手侧运行环境。
#
# 这个容器不是仿真栈的一部分（sim-world/flight-stack-nx01/flight-stack-nx02
# 那三个才是），它是选手自己`docker run --network host ...`起来的"一次性"
# 容器，跑完选手的任务脚本就退出（见2026大赛任务系统全流程任务仿真实现方案.md
# 第三部分方案①）。跟那三个容器一样用--network host，所以能直接共享宿主机
# 网络命名空间、看到仿真栈发布的ROS2话题。
#
# DDS配置（ROS_DOMAIN_ID/RMW_IMPLEMENTATION/CYCLONEDDS_URI三个环境变量+
# CycloneDDS锁lo回环那份配置文件）**不在这个脚本里做**，已经在
# Dockerfile.contestant-sdk里烘成镜像级ENV+COPY的静态文件了——具体原因见
# 那个Dockerfile里的详细注释：如果照抄flight-stack-entrypoint.sh"在entrypoint
# 脚本里运行时export"的写法，这几个变量只在这个脚本自己的PID1进程树里生效，
# 选手如果用VS Code Dev Containers打开这个镜像、在集成终端里用`docker exec`
# 进容器手动调试，是看不到这些变量的（`docker exec`只继承容器创建时的镜像
# ENV+`docker run -e`传的值，不继承entrypoint脚本运行时的export）——这正是
# 这次会话实测踩过的坑："docker exec"进容器不正确设置这两个环境变量+对应
# CycloneDDS配置，"ros2 topic list"会一直是空的。烘成镜像ENV之后，
# `docker run`和之后任意`docker exec`会话两条路径都能拿到同一份配置，不会
# 有一条路径漏掉。
set -eo pipefail

# 2026-09-21：镜像默认是仿真（21+锁lo回环）；连真机时由启动脚本传
# -e ROS_DOMAIN_ID=20 和换过的CYCLONEDDS_URI（见contestant_template/
# contestant_network.sh），这里按实际值回显，不再一律说"锁lo回环"。
case "${ROS_DOMAIN_ID:-}" in
    21) _mode="仿真" ;;
    20) _mode="真机" ;;
    *)  _mode="⚠️ 既不是仿真(21)也不是真机(20)，SDK会拒绝启动" ;;
esac
echo "== [contestant-sdk] ${_mode}：ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<未设置>} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<未设置>} CYCLONEDDS_URI=${CYCLONEDDS_URI:-<未设置>} =="

# 先不开-u：下面`source /opt/ros/humble/setup.bash`是ROS官方脚本，内部会引用
# 一些没有预先赋值的变量，跟set -u（nounset）不兼容，直接开-u source会崩——
# 跟flight-stack-entrypoint.sh/sim-world-entrypoint.sh同款写法，source完之后
# 再开-u，缩小nounset保护范围到"选手自己这段逻辑"，不覆盖ROS官方脚本本身。
source /opt/ros/humble/setup.bash
set -u

# 把容器启动命令的参数原样转发执行——选手用
# `docker run --rm --network host -v "$(pwd)":/workspace contestant-sdk:latest python3 /workspace/我的任务.py`
# 这种方式跑，"$@"此时就是`python3 /workspace/我的任务.py`整条命令，直接
# exec它本身即可，不需要（也不能）再在前面额外拼一个python3，否则等于把
# "python3"这个词当成第一个命令行参数传给了里面那层python3，变成
# `python3 python3 /workspace/我的任务.py`，会报"python3: can't open file
# 'python3'"。
exec "$@"
