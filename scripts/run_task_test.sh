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
docker run -d --name tk_leader --network host -v "$WORKDIR:/workspace" -v "$LOGDIR:/logs" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$LEADER" --role leader \
    --teammate "$FOLLOWER" --spacing "$SPACING" >/dev/null
docker run -d --name tk_follower --network host -v "$WORKDIR:/workspace" -v "$LOGDIR:/logs" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$FOLLOWER" --role follower \
    --teammate "$LEADER" --spacing "$SPACING" >/dev/null

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
