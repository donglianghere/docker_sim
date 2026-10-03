#!/usr/bin/env bash
# 后台起声光反馈常驻程序（contest_sdk.sound_light_server）：独立容器
# contestant-sound-light，用选手镜像contestant-sdk:latest，--restart
# unless-stopped——容器崩了/电脑重启后docker会自动拉起，真正"常驻待命"。
# 两架飞机的任务程序调sdk.play_sound_light()时只往/sound_light/request
# 话题发请求，由这个容器独占串口、排队发给声光板。配套关闭脚本见
# stop_sound_light_server.sh。
#
# 用法：
#   ./start_sound_light_server.sh                 # 串口默认/dev/ttyUSB0
#   ./start_sound_light_server.sh /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
#   ./start_sound_light_server.sh --dry-run       # 没接板子，只打印指令（仿真用）
#   ./start_sound_light_server.sh --real [串口]   # 真机调试：ROS_DOMAIN_ID=20
# 常驻程序必须跟任务程序在同一个域（仿真21/真机20）才收得到请求——地面站
# 在仿真/真机之间切换时，这个容器也要用对应模式重启一次。
# 看日志： docker logs -f contestant-sound-light
# 看状态： docker exec contestant-sound-light bash -lc \
#            "ros2 topic echo --once --qos-durability transient_local /sound_light/status"
# 手动测试（在任意选手容器里）：
#   ros2 topic pub --once /sound_light/request std_msgs/String "data: 侦察机起飞"
IMAGE_NAME="contestant-sdk:latest"
CONTAINER_NAME="contestant-sound-light"
set -eo pipefail

# 文件管理器里"右键 -> Run as a program"这种方式，是临时开一个终端窗口跑
# 这个脚本，脚本一结束终端窗口就自动关掉，看不清输出——用trap在退出前
# 停一下等按键，双击运行时才能看清结果再关；命令行直接跑也会多等这一下，
# 代价很小。
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

SERVER_ARGS=()
PORT="${SOUND_LIGHT_PORT:-/dev/ttyUSB0}"
MODE=sim
for arg in "$@"; do
    case "$arg" in
        --dry-run) SERVER_ARGS+=(--dry-run) ;;
        --real) MODE=real ;;
        *) PORT="$arg" ;;
    esac
done

if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "!! 找不到镜像 $IMAGE_NAME，先手动build（见docker/Dockerfile.contestant-sdk说明）。"
    exit 1
fi

# 镜像是改代码之前build的话里面没有常驻程序，提前说清楚，别让容器起来
# 之后在restart循环里反复崩。
if ! docker run --rm "$IMAGE_NAME" python3 -c "import contest_sdk.sound_light_server" >/dev/null 2>&1; then
    echo "!! 镜像 $IMAGE_NAME 里没有 contest_sdk.sound_light_server，是旧版镜像。"
    echo "   先重新build选手镜像（见 /home/robots/ai_uav/仿真编译 第二条命令）。"
    exit 1
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "容器 $CONTAINER_NAME 已经存在（可能还在跑，也可能上次没清理干净）。"
    echo "先用 stop_sound_light_server.sh 关掉旧的，再重新运行这个脚本。"
    exit 1
fi

if [[ ${#SERVER_ARGS[@]} -eq 0 && ! -e "$PORT" ]]; then
    echo "提示：$PORT 现在不存在（板子没插？），照样启动——常驻程序会每3秒重试，插上后自动连上。"
fi

# 仿真/真机两种网络参数（ROS_DOMAIN_ID 仿真21/真机20，真机还要换DDS网卡配置），
# 见 contestant_network.sh。
source /home/robots/ai_uav/docker_sim/scripts/contestant_network.sh
contestant_net_args "$MODE" /home/robots/ai_uav/docker_sim "$HOME/.cache/contest_sdk"
echo "运行模式：$CONTESTANT_NET_DESC"

# 串口直通不用--device：--device要求启动时设备已经存在，而且板子拔插后
# 设备节点会变，容器里看不到新节点。改成把/dev整个挂进去+按设备号放行
# （188=ttyUSB*，166=ttyACM*），拔插/改编号都不用重启容器。
docker run -d --name "$CONTAINER_NAME" --network host \
    --restart unless-stopped \
    -v /dev:/dev \
    --device-cgroup-rule='c 188:* rmw' \
    --device-cgroup-rule='c 166:* rmw' \
    -e SOUND_LIGHT_PORT="$PORT" \
    "${CONTESTANT_NET_ARGS[@]}" \
    "$IMAGE_NAME" \
    python3 -u -m contest_sdk.sound_light_server "${SERVER_ARGS[@]}" >/dev/null

sleep 3
docker logs "$CONTAINER_NAME" 2>&1 | tail -n 5
echo
echo "已启动声光反馈常驻程序：$CONTAINER_NAME（串口 ${SERVER_ARGS[*]:-$PORT}，$CONTESTANT_NET_DESC）"
echo "看日志： docker logs -f $CONTAINER_NAME"
echo "不用了跑 stop_sound_light_server.sh 关掉（否则开机会自动拉起）。"
