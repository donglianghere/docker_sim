#!/usr/bin/env bash
# 一次性运行选手容器：把 docker_sim/contestant_sim/我的任务.py 丢进
# contestant-sdk:latest 容器里跑一遍（等价于 contestant_sim/运行仿真.sh，
# 这里是给开发/联调用的顶层快捷方式）。
#
# 容器用--rm，跑完自动清理，不会越堆越多——这种"一次性"容器不需要额外的
# "关闭"脚本，运行结束它自己就没了。
#
# 这个脚本本身不放在docker_sim项目目录里，所以不能靠"脚本自己在哪个目录"
# 反推项目目录——直接写死绝对路径，不管这个脚本文件放在哪、从哪个当前
# 目录执行，都固定操作同一份任务代码。
#
# 用法： ./start_contestant_task.sh          # 调试仿真（ROS_DOMAIN_ID=21）
#        ./start_contestant_task.sh --real   # 调试真机（ROS_DOMAIN_ID=20）
TASK_DIR="/home/robots/ai_uav/docker_sim/contestant_sim"
TASK_FILE="我的任务.py"
IMAGE_NAME="contestant-sdk:latest"
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

if [[ ! -f "$TASK_DIR/$TASK_FILE" ]]; then
    echo "!! 找不到任务文件：$TASK_DIR/$TASK_FILE"
    exit 1
fi

# 仿真/真机两种网络参数（ROS_DOMAIN_ID 仿真21/真机20，真机还要换DDS网卡配置），
# 见 contestant_network.sh。
source /home/robots/ai_uav/docker_sim/scripts/contestant_network.sh
contestant_net_args "$MODE" /home/robots/ai_uav/docker_sim "$HOME/.cache/contest_sdk"
echo "运行模式：$CONTESTANT_NET_DESC"
echo "运行 $TASK_DIR/$TASK_FILE ..."
echo
docker run --rm --network host \
    -v "$TASK_DIR":/workspace \
    "${CONTESTANT_NET_ARGS[@]}" \
    "$IMAGE_NAME" \
    python3 "/workspace/$TASK_FILE"
