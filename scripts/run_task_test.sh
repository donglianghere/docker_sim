#!/usr/bin/env bash
# 跑单项任务测试（任务2/任务3 单项测试.py）：重启仿真 -> 等两机就绪 ->
# 起两个选手容器 -> 等两边都退出 -> 报结果。
#
#   ./scripts/run_task_test.sh --script 任务2单项测试.py
#   ./scripts/run_task_test.sh --script 任务3单项测试.py --keep
#
# 跟 run_formation_test.sh 的区别只有两处：跑哪个脚本可选；结束判据用
# **容器退出**而不是某一行日志——任务脚本没有统一的结束标志，长机和僚机
# 各自打各自的收尾话，靠 grep 一行字迟早对不上。
set -eo pipefail
cd "$(dirname "$0")/.."

SCRIPT="任务2单项测试.py"
SPACING=4.0
RESTART=1
KEEP=0
TIMEOUT_S=1500

while [ $# -gt 0 ]; do
    case "$1" in
        --script)     SCRIPT="$2"; shift 2 ;;
        --spacing)    SPACING="$2"; shift 2 ;;
        --no-restart) RESTART=0; shift ;;
        --keep)       KEEP=1; shift ;;
        --timeout)    TIMEOUT_S="$2"; shift 2 ;;
        -h|--help)    sed -n '2,9p' "$0"; exit 0 ;;
        *) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
    esac
done

LEADER=NX01
FOLLOWER=NX02
FSNX01=docker_sim-flight-stack-nx01-1
IMAGE=contestant-sdk:latest
WORKDIR="$PWD/contestant_template"
# 照片/日志目录：capture_photo() 默认往 /logs 写，不挂的话照片落在容器里，
# 容器一删就没了（"回传"至少要落到宿主机能看到的地方）。
LOGDIR="$PWD/runtime_logs"

log() { echo "[$(date +%H:%M:%S)] $*"; }
cleanup() { [ "$KEEP" = "1" ] && return 0; docker rm -f tk_leader tk_follower >/dev/null 2>&1 || true; }
trap cleanup EXIT

[ -f "$WORKDIR/$SCRIPT" ] || { echo "!! 找不到 $WORKDIR/$SCRIPT !!" >&2; exit 1; }
for img in "$IMAGE" flight-stack:latest sim-world:latest; do
    docker image inspect "$img" >/dev/null 2>&1 || { echo "!! 镜像 $img 不存在 !!" >&2; exit 1; }
done
docker rm -f tk_leader tk_follower >/dev/null 2>&1 || true

# ---- 起飞之前先过一遍静态自检 ----
# 2026-09-30：我改任务3 时从 `def _inspect_here` 一路切到 `def _back_to_observe_alt`，
# 把夹在中间的 center_fire_in_view / _drive_servos / _supply_point_action 三个
# 函数连带删掉了。py_compile 只查语法、照样通过，飞到"发现火情"那一步才抛
# NameError，两架飞机已经在天上了。仓库里本来就有这个按作用域查未定义名字的
# 检查器，能精确抓到这类错——问题不在工具缺失，在改完没跑。所以接进来，
# 让这一步跳不过去，不靠记性。
if [ -f scripts/check_python_static.py ]; then
    if ! python3 scripts/check_python_static.py contestant_template/*.py; then
        echo "!! 选手脚本静态自检不通过，先修好再飞 !!" >&2
        exit 1
    fi
fi

if [ "$RESTART" = "1" ]; then
    log "重启仿真容器…"
    docker compose down --timeout 20 >/dev/null 2>&1 || true
    docker compose up -d >/dev/null
fi

# 就绪判据跟 run_formation_test.sh 一样：ROS 节点 + PX4 自检都过。
# 不能写成 `docker logs | grep -q`——pipefail 下 grep 提前退出会把 docker logs
# 用 SIGPIPE 打死，匹配成功反被判为失败（那边的注释记着这条坑）。
log_has() { local out; out="$(docker logs "$1" 2>&1 || true)"; case "$out" in *"$2"*) return 0;; *) return 1;; esac; }
px4_ready() {
    for f in /tmp/px4_NX01.log /tmp/px4_NX02.log; do
        docker exec docker_sim-sim-world-1 grep -q "Ready for takeoff" "$f" 2>/dev/null || return 1
    done
}
log "等两机就绪（ROS节点 + PX4自检）…"
DL=$(( $(date +%s) + 240 ))
while :; do
    ok=1
    for c in nx01 nx02; do log_has "docker_sim-flight-stack-$c-1" 'formation_follower_node就绪' || ok=0; done
    px4_ready || ok=0
    [ "$ok" = "1" ] && break
    [ "$(date +%s)" -ge "$DL" ] && { echo "!! 等仿真就绪超时 !!" >&2; exit 1; }
    sleep 5
done
log "两机就绪"

log "启动选手程序：$SCRIPT（长机=$LEADER 僚机=$FOLLOWER 间距=${SPACING}米）"
docker run -d --name tk_leader --network host -v /etc/localtime:/etc/localtime:ro -v "$WORKDIR:/workspace" -v "$LOGDIR:/logs" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$LEADER" --role leader \
    --teammate "$FOLLOWER" --spacing "$SPACING" >/dev/null
docker run -d --name tk_follower --network host -v /etc/localtime:/etc/localtime:ro -v "$WORKDIR:/workspace" -v "$LOGDIR:/logs" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$FOLLOWER" --role follower \
    --teammate "$LEADER" --spacing "$SPACING" >/dev/null

# ---- 实时监视：跟测试同时开（用户 2026-09-30 要求"任务3 也要监控"）----
# 起在 flight-stack-nx01 容器里——那边的 DDS 环境是验证过的；显式传 DISPLAY，
# `docker exec` 起的新 shell 不一定继承得到（不给的话脚本会退回无窗口模式）。
MON_OUT="/logs/${SCRIPT%.py}_formation.png"
MON_LOG="/logs/${SCRIPT%.py}_monitor.log"
log "启动实时监视（窗口 + 报告 ${MON_OUT}）"
docker exec -d -e DISPLAY="${DISPLAY:-:1}" "$FSNX01" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0 \
           RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
           CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml
    python3 -u /opt/host_scripts/编队监视.py \
        --layout /opt/contest_mission_ws/src/contest_mission/config/sample_room_layout.yaml \
        --out '${MON_OUT}' --spacing ${SPACING} > '${MON_LOG}' 2>&1
" >/dev/null 2>&1 || echo "（监视没起来，不影响飞行测试）" >&2

DL=$(( $(date +%s) + TIMEOUT_S ))
while :; do
    live=0
    for c in tk_leader tk_follower; do docker ps --format '{{.Names}}' | grep -qx "$c" && live=$((live+1)); done
    [ "$live" = "0" ] && break
    if [ "$(date +%s)" -ge "$DL" ]; then
        echo "!! 超过 ${TIMEOUT_S} 秒仍未结束 !!" >&2
        for c in tk_leader tk_follower; do echo "--- $c:" >&2; docker logs --tail 8 "$c" 2>&1 >&2; done
        exit 1
    fi
    sleep 5
done

# 选手程序都退了，让监视收尾出图：它自己的"任务结束"判据是给编队飞行写的
# （两机都落地/悬停够久），任务流程里不一定成立；这里直接发 SIGINT，脚本
# 收到就存 PNG+CSV 再退。给几秒让它写完。
docker exec "$FSNX01" pkill -INT -f 编队监视 >/dev/null 2>&1 || true
sleep 6

# 属主修正：选手容器以 root 身份往 /logs 写（照片、日志），宿主机这边属主就是
# root，普通用户删不掉也改不了。跑完借一个一次性容器把属主改回来——宿主机不需要
# sudo，改的是同一个挂载点。
docker run --rm -v "$LOGDIR:/logs" --entrypoint chown "$IMAGE" \
    -R "$(id -u):$(id -g)" /logs >/dev/null 2>&1 || \
    echo "（属主修正没做成，$LOGDIR 下的新文件属主可能是 root）" >&2

echo ""
echo "================ $SCRIPT 结果 ================"
rc=0
for c in tk_leader tk_follower; do
    code="$(docker inspect -f '{{.State.ExitCode}}' "$c" 2>/dev/null || echo '?')"
    echo "--- $c 退出码 $code"
    [ "$code" = "0" ] || rc=1
    docker logs "$c" 2>&1 | grep -E 'Traceback|Error|错误|失败' | tail -5 || true
done
echo "=============================================="
exit "$rc"
