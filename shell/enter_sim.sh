#!/usr/bin/env bash
# 进仿真容器的命令行。三个容器选一个：
#
#   ./enter_sim.sh nx01                  # flight-stack-nx01（默认）
#   ./enter_sim.sh nx02                  # flight-stack-nx02
#   ./enter_sim.sh world                 # sim-world（gzserver/PX4 那个）
#   ./enter_sim.sh nx01 "ros2 node list" # 跑一条就退出
#
# ⚠️ 跟 enter_contestant.sh / enter_sound_light.sh 不同，这里**必须**额外
# source ros2_env_setup.sh：flight-stack 的 entrypoint 按 PLANNER 联动 export
# RMW_IMPLEMENTATION / CYCLONEDDS_URI，那个 export 只在 entrypoint 自己的
# 进程树里有效，docker exec 开的新 shell 读不到（容器级变量里只有 PLANNER
# 本身）。不补的话 PLANNER=ego_planner 时你这个 shell 还是默认 FastDDS，
# 跟已经切到 CycloneDDS 的真实节点完全对不上话题——`ros2 topic list` 空的，
# 看着像连不上，实际只是 RMW 不一致。同一个坑 scripts/exec_gcs.sh 和
# scripts/ros2_env_setup.sh 的文件头都记过。
#
# ⚠️ 查话题一律加 --no-daemon。`ros2 topic list` 默认问 ROS 2 daemon，而
# daemon 是**按 (域, RMW) 起一次就常驻**的：只要之前有谁在这个容器里用对的
# RMW 起过它，后面不管你自己的 RMW 对不对，`ros2 topic list` 都会读那份缓存、
# 返回完整列表。于是一个 RMW 配错的 shell 看起来完全正常——2026-10-03 实测
# 踩过：裸 docker exec（默认 FastDDS）也返回 465 条，差点据此断定本脚本
# source ros2_env_setup.sh 那步可省。`ros2 daemon stop` 清掉缓存后两边加
# --no-daemon 重测才是真值：FastDDS 2 条 / CycloneDDS 465 条。
# （vision_real.sh / run_real.sh 的就绪判据本来就都用 --no-daemon。）
#
# flight-stack 两个容器把 ./scripts 挂成了 /opt/host_scripts（compose 里
# `./scripts:/opt/host_scripts:ro`），直接 source 那份就行、不用 docker cp。
# sim-world **没有**这个挂载，所以那一路先 docker cp 一份到 /tmp。
set -eo pipefail
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-nx01}" in
    nx01)  C=docker_sim-flight-stack-nx01-1 ;;
    nx02)  C=docker_sim-flight-stack-nx02-1 ;;
    world) C=docker_sim-sim-world-1 ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "!! 不认识的目标：$1（可选 nx01 / nx02 / world）" >&2; exit 2 ;;
esac
shift || true

docker ps --format '{{.Names}}' | grep -qx "$C" || {
    echo "!! 容器 $C 没在跑" >&2
    echo "   先起仿真： $(dirname "${BASH_SOURCE[0]}")/start_sim.sh" >&2
    exit 1
}

# RMW 环境：挂载有就用挂载的，没有就 cp 一份（sim-world 走这条）
if docker exec "$C" test -f /opt/host_scripts/ros2_env_setup.sh 2>/dev/null; then
    ENVSH=/opt/host_scripts/ros2_env_setup.sh
else
    docker cp "$ROOT/scripts/ros2_env_setup.sh" "$C:/tmp/ros2_env_setup.sh" >/dev/null
    ENVSH=/tmp/ros2_env_setup.sh
fi

p=$(docker exec "$C" printenv PLANNER 2>/dev/null || echo "?")
d=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$C" | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
echo "== $C   域 ${d:-未知}   PLANNER=$p   RMW 由 $ENVSH 推导 =="
echo "   查话题加 --no-daemon： ros2 topic list --no-daemon"
echo "   （不加会读 ROS 2 daemon 的缓存，RMW 配错也看着正常；见文件头）"

PRE="source /opt/ros/humble/setup.bash && source $ENVSH"
# 交互式才要 -it；"跑一条命令"的形式不加 -t，否则非 TTY 环境（管道、
# 别的脚本里调）会报 "cannot attach stdin to a TTY-enabled container"。
if [ $# -eq 0 ]; then
    exec docker exec -it "$C" bash -lc "$PRE; exec bash"
else
    exec docker exec "$C" bash -lc "$PRE && $*"
fi
