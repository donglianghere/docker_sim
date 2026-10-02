#!/usr/bin/env bash
# 一键跑单机测试。跟 run_real.sh 共用同一套前置闸门，但**只起一架**。
#
#   ./run_test.sh t1              # 物资抓取（默认 NX02，带夹爪）
#   ./run_test.sh t2              # 地面火情对准（默认 NX01）
#   ./run_test.sh t3 --n          # 高层火情瞄准，用 N 点看 1# 楼
#   ./run_test.sh t4 --nx02       # 避障，指定用 NX02
#   ./run_test.sh t5 --keep       # 仿地，结束后保留容器看日志
#   ./run_test.sh                 # 不给就列出可选的测试
#
# 默认机号是按测试内容定的：t1 要夹爪所以默认 NX02（任务机），其余默认 NX01。
# 用 --nx01 / --nx02 可以覆盖。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TDIR="$HERE/单机测试"
IMAGE=contestant-sdk:latest
LOGDIR="$ROOT/runtime_logs"
C_TEST=real_test
C_MONITOR=real_test_monitor
NX01_IP=192.168.2.101
NX02_IP=192.168.2.102
TIMEOUT_S=900
KEEP=0
NS=""
EXTRA=()

log() { echo "[$(date +%H:%M:%S)] $*"; }
die() { echo "!! $*" >&2; exit 1; }

# 同 run_real.sh：容器名固定，上次 --keep / 崩了 / Ctrl-C 都会留下容器，
# 这次 docker run --name 会撞名起不来。启动前清一遍 + trap 保证 Ctrl-C 也清。
cleanup() { [ "${KEEP:-0}" = "1" ] && return 0
            docker rm -f "$C_TEST" "$C_MONITOR" >/dev/null 2>&1 || true; }
trap cleanup EXIT
purge_stale() {
    local n stale=""
    for n in "$C_TEST" "$C_MONITOR"; do
        docker ps -a --format '{{.Names}}' | grep -qx "$n" && stale="$stale $n"
    done
    [ -z "$stale" ] && return 0
    log "清理上一次残留的容器：$stale"
    docker rm -f $stale >/dev/null 2>&1 || true
}

list_tests() {
    echo "可选的测试："
    for f in "$TDIR"/t*.py; do
        [ -e "$f" ] || continue
        b="$(basename "$f" .py)"
        desc="$(grep -m1 '^"""单机测试' "$f" | sed 's/^"""//;s/。$//')"
        printf '  %-16s %s\n' "${b%%_*}" "$desc"
    done
    echo
    echo "用法： ./run_test.sh t1 [--nx01|--nx02] [--keep] [--timeout 秒] [程序自己的参数...]"
}

[ $# -eq 0 ] && { list_tests; exit 0; }
WHICH="$1"; shift
while [ $# -gt 0 ]; do
    case "$1" in
        --nx01) NS=NX01; shift ;;
        --nx02) NS=NX02; shift ;;
        --keep) KEEP=1; shift ;;
        --timeout) TIMEOUT_S="$2"; shift 2 ;;
        -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
        *) EXTRA+=("$1"); shift ;;     # 其余原样转给测试程序（比如 t3 的 --n）
    esac
done

# ---- 找程序 ----
SCRIPT="$(ls "$TDIR"/${WHICH}_*.py 2>/dev/null | head -1)"
[ -n "$SCRIPT" ] || { echo "!! 找不到测试 '$WHICH'" >&2; echo >&2; list_tests >&2; exit 2; }
BASENAME="$(basename "$SCRIPT")"
# t1 要夹爪 -> 默认任务机；其余默认侦察机
[ -n "$NS" ] || { case "$WHICH" in t1) NS=NX02 ;; *) NS=NX01 ;; esac; }
case "$NS" in NX01) IP=$NX01_IP ;; NX02) IP=$NX02_IP ;; esac
log "测试：$BASENAME    飞机：$NS ($IP)"

# ---- 互斥：跟 run_real.sh 共用一把锁，不能同时飞 ----
LOCK=/tmp/docker_sim_run_real.lock
exec 9>"$LOCK"
flock -n 9 || die "已经有 run_real.sh 或另一个 run_test.sh 在跑"

# ---- 前置闸门（跟 run_real.sh 同一套，只查这一架）----
purge_stale
"$ROOT/scripts/check_env.sh" real || die "环境检查未通过"

ping -c1 -W2 "$IP" >/dev/null 2>&1 || die "飞机 $IP 不可达"
st="$(curl -s --noproxy '*' --max-time 8 "http://$IP:8890/status" 2>/dev/null)"
[ -n "$st" ] || die "飞机 $IP 的 control_server 没响应"
ns_rep="$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("namespace",""))' 2>/dev/null)"
fu="$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("flight_up"))' 2>/dev/null)"
vu="$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("vision_up"))' 2>/dev/null)"
[ "$ns_rep" = "$NS" ] || die "$IP 报的命名空间是 $ns_rep，期望 $NS"
[ "$fu" = "True" ] || die "$IP 的 flight-stack 没起"
[ "$vu" = "True" ] || die "$IP 的 vision-stack 没起"
log "机载栈就绪：$ns_rep  flight_up=$fu  vision_up=$vu"

# 视觉：t4/t5 不用视觉，其余三个必须有
case "$WHICH" in
  t4|t5) log "本测试不依赖视觉，跳过视觉就绪检查" ;;
  *)
    log "检查视觉就绪"
    # shellcheck source=/dev/null
    source "$ROOT/scripts/contestant_network.sh"
    contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" || die "网络参数生成失败"
    n="$(timeout 90 docker run --rm --network host "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
         bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null
                   n=0; for i in $(seq 1 15); do
                       n=$(timeout 6 ros2 topic list --no-daemon 2>/dev/null | grep -c "'"$NS"'/vision/detections")
                       [ "$n" -ge 1 ] && break
                       sleep 2
                   done; echo "$n"' \
         2>/dev/null | tr -d '\r' | grep -E '^[0-9]+$' | tail -1 || echo 0)"
    [ "${n:-0}" -ge 1 ] || die "$NS 的 vision/detections 没有发布者——先 ./vision_real.sh up"
    log "视觉就绪"
    ;;
esac
# t4/t5 跳过了上面的 source，这里补上
[ ${#CONTESTANT_NET_ARGS[@]} -gt 0 ] 2>/dev/null || {
    source "$ROOT/scripts/contestant_network.sh"
    contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" || die "网络参数生成失败"
}
log "网络：$CONTESTANT_NET_DESC"

# ---- 监视（单机也开：高度曲线是 t5 的主要判据）----
MON_OUT="/logs/${BASENAME%.py}_report.png"
log "启动监视窗口（报告存 runtime_logs/$(basename "$MON_OUT")）"
docker run -d --name "$C_MONITOR" --network host \
    -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -v "$ROOT/scripts:/scripts:ro" -v "$LOGDIR:/logs" \
    "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    python3 -u /scripts/monitor.py --leader "$NS" --follower "$NS" \
        --out "$MON_OUT" >/dev/null 2>&1 \
    || echo "（监视没起来，不影响飞行）" >&2

# ---- 跑 ----
BASE_PYPATH="$(docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$IMAGE" \
               | sed -n 's/^PYTHONPATH=//p' | head -1)"
PYPATH="/workspace${BASE_PYPATH:+:$BASE_PYPATH}"; PYPATH="${PYPATH%:}"
log "启动测试程序"
docker run -d --name "$C_TEST" --network host \
    -v /etc/localtime:/etc/localtime:ro \
    -v "$HERE:/workspace:ro" -v "$LOGDIR:/logs" \
    -e PYTHONPATH="$PYPATH" "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    python3 -u "/workspace/单机测试/$BASENAME" \
      --namespace "$NS" --role leader --teammate "$NS" "${EXTRA[@]}" >/dev/null

echo
log "测试中……（实时日志：docker logs -f $C_TEST）"
DL=$(( $(date +%s) + TIMEOUT_S ))
while docker ps --format '{{.Names}}' | grep -qx "$C_TEST"; do
    if [ "$(date +%s)" -ge "$DL" ]; then
        echo "!! 超过 ${TIMEOUT_S} 秒未结束 !!" >&2
        docker logs --tail 15 "$C_TEST" >&2 2>&1
        exit 1
    fi
    sleep 5
done

# ---- 收尾 ----
docker exec "$C_MONITOR" pkill -INT -f monitor.py >/dev/null 2>&1 || true
sleep 5
docker run --rm -v "$LOGDIR:/logs" --entrypoint chown "$IMAGE" \
    -R "$(id -u):$(id -g)" /logs >/dev/null 2>&1 || true

echo
echo "================ $BASENAME 结果 ================"
code="$(docker inspect -f '{{.State.ExitCode}}' "$C_TEST" 2>/dev/null || echo '?')"
echo "退出码 $code"
docker logs "$C_TEST" 2>&1 | grep -E 'Traceback|Error|错误|失败|!!' | tail -8 || true
echo
echo "报告图：runtime_logs/$(basename "$MON_OUT")"
echo "机载栈仍在运行（本脚本不拆机载容器）。"

[ "$KEEP" = "0" ] && docker rm -f "$C_TEST" "$C_MONITOR" >/dev/null 2>&1 || \
    echo "（--keep：容器已保留，看完 docker rm -f $C_TEST $C_MONITOR）"
exit "${code:-1}"
