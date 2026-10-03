#!/usr/bin/env bash
# 进选手调试容器的命令行（contestant-sdk-shell）。
#
#   ./enter_contestant.sh                   # 交互式 bash，进去就能跑 ros2 CLI
#   ./enter_contestant.sh "ros2 topic list" # 跑一条就退出
#
# 容器由 start_contestant_shell.sh 起（它决定域号：默认仿真 21，--real 真机 20）。
# 本脚本不碰域号，也不起容器——进哪个域由它当初怎么起的决定，所以进来先打出
# 域号给你看。
#
# 为什么要 source：镜像里 ros2 不在默认 PATH 上（实测 `command -v ros2` 为空），
# 交互式 bash 不读那串 setup.bash。而 ROS_DOMAIN_ID / RMW_IMPLEMENTATION /
# CYCLONEDDS_URI **不用**在这里补——三个都是 docker run -e 传的容器级变量
# （见 scripts/contestant_network.sh 的 CONTESTANT_NET_ARGS）或镜像 ENV，
# docker exec 开的新 shell 天然读得到。这点和 flight-stack 容器不同，那边
# RMW 是 entrypoint 运行时 export 的，必须靠 ros2_env_setup.sh 重新推导。
set -eo pipefail
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

C=contestant-sdk-shell
docker ps --format '{{.Names}}' | grep -qx "$C" || {
    echo "!! 容器 $C 没在跑" >&2
    echo "   先起它： $(dirname "${BASH_SOURCE[0]}")/start_contestant_shell.sh        # 仿真 21 域" >&2
    echo "            $(dirname "${BASH_SOURCE[0]}")/start_contestant_shell.sh --real # 真机 20 域" >&2
    exit 1
}
d=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$C" | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
case "$d" in
    20) echo "== $C   域 20 → 真机 ==" ;;
    21) echo "== $C   域 21 → 仿真 ==" ;;
    *)  echo "== $C   域 ${d:-未知} ==" ;;
esac

# 交互式才要 -it；"跑一条命令"的形式不加 -t，否则非 TTY 环境（管道、
# 别的脚本里调）会报 "cannot attach stdin to a TTY-enabled container"。
if [ $# -eq 0 ]; then
    exec docker exec -it "$C" bash -lc 'source /opt/ros/humble/setup.bash; exec bash'
else
    exec docker exec "$C" bash -lc "source /opt/ros/humble/setup.bash && $*"
fi
