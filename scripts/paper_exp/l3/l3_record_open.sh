#!/usr/bin/env bash
# 不限时录制：start 开录（后台常驻），stop 优雅停止。用于真实飞行这种
# "时长不定、叫停再停"的场景——`l3_record.sh` 是固定时长的，不适合。
#
# 用法（容器内）：
#   ./l3_record_open.sh start <标签> [命名空间] [--lock]
#   ./l3_record_open.sh stop
#   ./l3_record_open.sh status
set -o pipefail
ACT="${1:?用法: l3_record_open.sh start <标签> [ns] [--lock] | stop | status}"
source /opt/ros/humble/setup.bash
source /opt/px4ctrl_ws/install/setup.bash 2>/dev/null || true
for pat in origin_setter dlio_odom_node mavros_node; do
    PID=$(pgrep -f "${pat}" | head -1); [ -z "${PID}" ] && continue
    [ -r "/proc/${PID}/environ" ] || continue
    eval "$(tr '\0' '\n' < /proc/${PID}/environ | grep -E '^(RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_DOMAIN_ID)=' | sed 's/^/export /')"
    break
done

case "${ACT}" in
stop)
    # ros2 bag record 只认 SIGINT 做优雅收尾（写 metadata.yaml），不能用 TERM
    pkill -INT -f "ros2 bag record" && echo "已发送停止信号，等待收尾..." || echo "没有在跑的录制"
    for i in $(seq 1 20); do pgrep -f "ros2 bag record" >/dev/null || break; sleep 1; done
    pgrep -f "ros2 bag record" >/dev/null && echo "⚠️ 20秒后仍未退出" || echo "录制已停止"
    ls -la /logs/l3/ 2>/dev/null | tail -3
    ;;
status)
    if pgrep -f "ros2 bag record" >/dev/null; then
        echo "录制中: $(ps -eo args | grep -o '\-o /logs/l3/[^ ]*' | head -1)"
        du -sh /logs/l3/* 2>/dev/null | tail -2
    else
        echo "当前没有录制在跑"
    fi
    ;;
start)
    TAG="${2:?缺标签}"; NS="${3:-NX01}"; OUT="/logs/l3/${TAG}"
    for a in "$@"; do [ "$a" = "--lock" ] && LOCK=1; done
    pgrep -f "ros2 bag record" >/dev/null && { echo "!! 已有录制在跑，先 stop"; exit 1; }
    [ -e "${OUT}" ] && { echo "!! ${OUT} 已存在，换个标签"; exit 1; }
    TOPICS=""
    for t in dlio/odom_node/odom uwb/pose_abs uwb_a/pose_abs uwb_b/pose_abs \
             origin_setter/yaw_estimate origin_setter/yaw_sample_count origin_locked \
             mavros/state mavros/imu/data mavros/local_position/pose \
             mavros/setpoint_raw/target_attitude term_goal; do
        TOPICS="$TOPICS /${NS}/${t}"
    done
    TOPICS="$TOPICS /tf /tf_static"
    echo "== 话题自检 =="
    LIVE=$(ros2 topic list 2>/dev/null)
    for t in ${TOPICS}; do echo "$LIVE" | grep -qx "$t" && echo "   [有] $t" || echo "   [缺] $t"; done
    if [ -n "${LOCK:-}" ]; then
        echo "== 锁定起飞点（确认飞机静止在起飞点上）=="
        ros2 service call "/${NS}/set_origin_from_uwb" std_srvs/srv/Trigger 2>&1 | tail -2
        sleep 1
    fi
    mkdir -p /logs/l3
    echo "== 开始不限时录制 -> ${OUT} =="
    setsid nohup ros2 bag record -o "${OUT}" ${TOPICS} < /dev/null > "/tmp/l3_bag_${TAG}.log" 2>&1 &
    sleep 6
    pgrep -f "ros2 bag record" >/dev/null && echo "录制已启动，用 stop 停止" || { echo "!! 启动失败"; tail -10 "/tmp/l3_bag_${TAG}.log"; }
    ;;
*) echo "未知动作: ${ACT}"; exit 1 ;;
esac
