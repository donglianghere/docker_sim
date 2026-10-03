#!/usr/bin/env bash
# 后台起一个常驻的选手容器（命名固定为contestant-sdk-shell，不用--rm），
# 方便进去手动跑命令/调试contest_sdk，不是"跑完就走"的一次性任务——
# 跟 start_contestant_task.sh 是两种不同的用法，配套的关闭脚本见
# stop_contestant_shell.sh。
#
# 这个脚本本身不放在docker_sim项目目录里，所以不能靠"脚本自己在哪个目录"
# 反推项目目录——直接写死绝对路径，不管这个脚本文件放在哪、从哪个当前
# 目录执行，都固定挂载同一份任务代码目录。
#
# 用法： ./start_contestant_shell.sh          # 调试仿真（ROS_DOMAIN_ID=21）
#        ./start_contestant_shell.sh --real   # 调试真机（ROS_DOMAIN_ID=20）
TASK_DIR="/home/robots/ai_uav/docker_sim/contestant_sim"
IMAGE_NAME="contestant-sdk:latest"
CONTAINER_NAME="contestant-sdk-shell"
set -eo pipefail

# 文件管理器里"右键 -> Run as a program"这种方式，是临时开一个终端窗口跑
# 这个脚本，脚本一结束终端窗口就自动关掉，看不清输出——用trap在退出前
# 停一下等按键，双击运行时才能看清结果再关；命令行直接跑也会多等这一下，
# 代价很小。
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

MODE=sim
[[ "${1:-}" == "--real" ]] && MODE=real

if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "!! 找不到镜像 $IMAGE_NAME，先手动build（见docker/Dockerfile.contestant-sdk说明）。"
    exit 1
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "容器 $CONTAINER_NAME 已经存在（可能还在跑，也可能上次没清理干净）。"
    echo "先用 stop_contestant_shell.sh 关掉旧的，再重新运行这个脚本。"
    exit 1
fi

# 仿真/真机两种网络参数（ROS_DOMAIN_ID 仿真21/真机20，真机还要换DDS网卡配置），
# 见 contestant_network.sh。
source /home/robots/ai_uav/docker_sim/scripts/contestant_network.sh
contestant_net_args "$MODE" /home/robots/ai_uav/docker_sim "$HOME/.cache/contest_sdk"
echo "运行模式：$CONTESTANT_NET_DESC"

docker run -d --name "$CONTAINER_NAME" --network host \
    -v "$TASK_DIR":/workspace \
    "${CONTESTANT_NET_ARGS[@]}" \
    "$IMAGE_NAME" \
    sleep infinity

echo "已启动常驻选手容器：$CONTAINER_NAME"
echo "进去调试命令： docker exec -it $CONTAINER_NAME bash"
echo "不用了记得跑 stop_contestant_shell.sh 关掉。"
