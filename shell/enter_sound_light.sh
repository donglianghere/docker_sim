#!/usr/bin/env bash
# 进声光常驻容器的命令行（contestant-sound-light）。
#
#   ./enter_sound_light.sh                                     # 交互式 bash
#   ./enter_sound_light.sh "ros2 topic echo --once /sound_light/status"
#
# 想让装置响不用进来，用 test_sound_light.sh 就行。进来主要是看话题、
# 看串口、手工发原始请求。
#
# 环境同 enter_contestant.sh：只需 source setup.bash（镜像里 ros2 不在默认
# PATH），域号/RMW/CYCLONEDDS_URI 都是容器级 env，docker exec 读得到。
set -eo pipefail
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

C=contestant-sound-light
docker ps --format '{{.Names}}' | grep -qx "$C" || {
    echo "!! 容器 $C 没在跑" >&2
    echo "   先起它： $(dirname "${BASH_SOURCE[0]}")/start_sound_light_server.sh [--real] /dev/ttyUSB0" >&2
    exit 1
}
d=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$C" | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
case "$d" in
    20) echo "== $C   域 20 → 真机 ==" ;;
    21) echo "== $C   域 21 → 仿真 ==" ;;
    *)  echo "== $C   域 ${d:-未知} ==" ;;
esac
echo "   常用： ros2 topic echo --once --full-length /sound_light/status"
echo "          ros2 topic pub --once /sound_light/request std_msgs/String 'data: 侦察机起飞'"

# 交互式才要 -it；"跑一条命令"的形式不加 -t，否则非 TTY 环境（管道、
# 别的脚本里调）会报 "cannot attach stdin to a TTY-enabled container"。
if [ $# -eq 0 ]; then
    exec docker exec -it "$C" bash -lc 'source /opt/ros/humble/setup.bash; exec bash'
else
    exec docker exec "$C" bash -lc "source /opt/ros/humble/setup.bash && $*"
fi
