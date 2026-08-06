#!/usr/bin/env bash
# 一键启动脚本：先给容器授权访问宿主机X server（GUI用），再 docker compose up。
#
# 为什么需要这一步：xhost授权是宿主机X server的运行时ACL，只在当前X session
# 存活期间有效（重启图形会话/宿主机后会失效），docker-compose.yml本身没有
# 能在`up`之前自动执行宿主机命令的钩子，所以没法把xhost这条命令写进compose
# 文件里，只能在外面包一层脚本。
set -eo pipefail
cd "$(dirname "$0")"

if [[ -z "${DISPLAY:-}" ]]; then
    echo "警告：当前没有 DISPLAY 环境变量，Gazebo/RViz2 的GUI窗口不会显示（仍会正常仿真）。" >&2
else
    echo "== 授权容器访问宿主机 X server (xhost +local:docker) =="
    xhost +local:docker
fi

# 处理容器已经在跑的情况：如果sim-world已经起来了，说明它的gzclient/rviz2这两个
# GUI子进程是在上面的xhost授权之前就启动的，那时候大概率没被授权、已经崩溃退出了
# （entrypoint里`wait ${GAZEBO_PID}`只等gzserver，gzclient/rviz2挂了不会拖垮整个
# 容器，容器本身会显示"running"，但GUI窗口其实早就没了，`docker compose up`此时
# 不会重新拉起它们——因为容器配置没变，compose认为"不需要重建"）。只补xhost授权、
# 不重启容器的话，已经死掉的GUI进程不会自己复活，所以这里检测到sim-world已在跑
# 就重启它。sim-world/flight-stack-nx01/flight-stack-nx02三个容器是一体的
# （flight-stack靠depends_on等sim-world、且DLIO/mighty/MAVROS都要连sim-world里
# 的PX4 SITL+点云+IMU等话题），单独重启sim-world会导致它的Gazebo/PX4实例重新
# 初始化，两个flight-stack容器里缓存的旧连接/订阅状态就跟新的sim-world脱节了，
# 所以只要sim-world需要重启，就把三个一起重启，保证三者的运行时状态重新对齐。
RUNNING_SERVICES=$(docker compose ps --status running --services 2>/dev/null || true)
if echo "$RUNNING_SERVICES" | grep -qx "sim-world"; then
    echo "== 检测到 sim-world 容器已在运行，三个容器是一体的，一起重启以重新拉起 gzclient/rviz2（应用刚才的X server授权） =="
    docker compose restart sim-world flight-stack-nx01 flight-stack-nx02
fi

echo "== docker compose up -d =="
docker compose up -d "$@"

# 前台docker compose up会把sim-world/flight-stack-nx01/flight-stack-nx02三份
# 日志全部糊在一个终端里，谁的日志都不好找。改成后台起容器，再交给
# watch_sim.sh拉起tmux三窗格分开看（脚本本身在scripts/up_and_watch.sh基础上
# 内联过来，避免多一层脚本调用，行为完全一致）。
exec ./scripts/watch_sim.sh
