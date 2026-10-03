#!/usr/bin/env bash
# 一键在**仿真**里跑单机测试。跟 contestant_real/run_test.sh 是一对：
# 同一套测试程序（逐字节相同），坐标各取自己目录的 venue.py。
#
#   ./run_test.sh t1              # 物资抓取（默认 NX02，带夹爪）
#   ./run_test.sh t2              # 地面火情对准（默认 NX01）
#   ./run_test.sh t3 --n          # 高层火情，用 N 点看 1# 楼
#   ./run_test.sh t4 --nx02       # 避障，指定用 NX02
#   ./run_test.sh t5 --keep       # 仿地，结束后保留容器看日志
#   ./run_test.sh t1 --restart    # 强制重起仿真（默认复用已在跑的）
#   ./run_test.sh                 # 不给就列出可选的测试
#
# 为什么不能用 run.sh 跑这些：run.sh **无条件起两架**（sim_leader +
# sim_follower 跑同一个程序），而这五个测试都是 DroneSDK.run(leader=test,
# follower=test)——两架会同时飞向同一个点（t1 两架都去物资点，t2/t3 都去同一
# 个火情位）。不是安全问题，是测试本身没意义。所以单机测试要有自己的运行器。
#
# 和真机 run_test.sh 的区别：
#   · 仿真栈归本脚本管（默认复用已在跑的，--restart 才 compose down+up）；
#     真机那边机载栈归各机 control_server 管，只探测不代起。
#   · 就绪判据等 PX4 "Ready for takeoff" + flight-stack 节点；真机等 8890
#     /status 的 flight_up/vision_up。
#   · 不需要 vision_real.sh 那一步——仿真的检测节点随 flight-stack 一起起。
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TDIR="$HERE/单机测试"
IMAGE=contestant-sdk:latest
LOGDIR="$ROOT/runtime_logs"
# 监视的底图布局。run.sh 用的是容器内路径
# /opt/contest_mission_ws/src/.../sample_room_layout.yaml——那是 **flight-stack**
# 镜像里才有的（run.sh 的监视是 docker exec 进那个容器跑的）。本脚本的监视跑在
# contestant-sdk 容器里，**没有**那个路径，直接传过去会让 monitor.py 退回写死的
# 老坐标、俯视图的房间/立柱位置跟实际场景对不上（2026-10-03 实测踩到，监视日志
# 里有"布局 ... 读不了"那一行）。所以从宿主机挂一份进去。
LAYOUT_HOST="$ROOT/src/contest_mission/config/sample_room_layout.yaml"
LAYOUT=/layout.yaml
SIMWORLD=docker_sim-sim-world-1
C_TEST=sim_test
C_MONITOR=sim_test_monitor
TIMEOUT_S=900
KEEP=0; NS=""; RESTART=0; EXTRA=()

log() { echo "[$(date +%H:%M:%S)] $*"; }
die() { echo "!! $*" >&2; exit 1; }

# 前一次的残余：容器名固定，上次若用了 --keep、或崩了/被 Ctrl-C 掉，容器会
# 留着，这次 docker run --name 会撞名起不来。启动前清一遍 + trap 保证
# Ctrl-C 也清（跟 run.sh / 真机 run_test.sh 同样的两层）。
cleanup() { [ "$KEEP" = "1" ] && return 0
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
    echo "可选的测试（仿真）："
    for f in "$TDIR"/t*.py; do
        [ -e "$f" ] || continue
        b="$(basename "$f")"
        printf "  %-6s %s\n" "${b%%_*}" "$b"
    done
    echo
    echo "默认机号：t1=NX02（要夹爪），其余=NX01。用 --nx01/--nx02 覆盖。"
}

[ $# -ge 1 ] || { list_tests; exit 0; }
case "$1" in -h|--help) sed -n '2,30p' "$0"; exit 0 ;; esac
WHICH="$1"; shift
while [ $# -gt 0 ]; do
    case "$1" in
        --nx01)    NS=NX01; shift ;;
        --nx02)    NS=NX02; shift ;;
        --keep)    KEEP=1; shift ;;
        --restart) RESTART=1; shift ;;
        --timeout) TIMEOUT_S="$2"; shift 2 ;;
        *) EXTRA+=("$1"); shift ;;     # 其余原样转给测试程序（比如 t3 的 --n）
    esac
done

SCRIPT="$(ls "$TDIR"/${WHICH}_*.py 2>/dev/null | head -1)"
[ -n "$SCRIPT" ] || { echo "!! 找不到测试 '$WHICH'" >&2; echo >&2; list_tests >&2; exit 2; }
BASENAME="$(basename "$SCRIPT")"
[ -n "$NS" ] || { case "$WHICH" in t1) NS=NX02 ;; *) NS=NX01 ;; esac; }
log "测试：$BASENAME    飞机：$NS（仿真）"

LOCK=/tmp/docker_sim_run.lock
exec 9>"$LOCK"
flock -n 9 || die "已经有一个 run.sh / run_test.sh 在跑（锁 $LOCK）"

purge_stale

# ---- 共享栈必起（同 run.sh：声光必起，网页栈不管）----
source "$ROOT/scripts/ensure_stacks.sh"
ensure_sound_light sim

"$ROOT/scripts/check_env.sh" sim || die "环境检查未通过（见上），处理后再跑"

# ---- 静态自检 ----
if [ -f "$ROOT/scripts/check_python_static.py" ]; then
    python3 "$ROOT/scripts/check_python_static.py" "$SCRIPT" "$HERE/venue.py" \
        || die "静态自检不通过，先修好再跑"
fi

# ---- 仿真栈：默认复用，--restart 才重起 ----
sim_running() { docker ps --format '{{.Names}}' | grep -qx "$SIMWORLD"; }
if [ "$RESTART" = "1" ] || ! sim_running; then
    [ "$RESTART" = "1" ] && log "按 --restart 重起仿真容器…" || log "仿真没在跑，起它…"
    ( cd "$ROOT" && WORLD_ENV=sample_room docker compose down --timeout 20 >/dev/null 2>&1 || true )
    ( cd "$ROOT" && WORLD_ENV=sample_room docker compose up -d >/dev/null )
else
    log "复用已在跑的仿真栈（要重起加 --restart）"
fi

# 不能写成 `docker logs | grep -q`：pipefail 下 grep 命中就退出，docker logs
# 还在往管道写、被 SIGPIPE 打死(141)，匹配成功反而判成失败，日志越大越必然。
log_has() { local out; out="$(docker logs "$1" 2>&1 || true)"; case "$out" in *"$2"*) return 0 ;; *) return 1 ;; esac; }
log "等 $NS 就绪（ROS 节点 + PX4 自检）…"
c="docker_sim-flight-stack-$(echo "$NS" | tr 'A-Z' 'a-z')-1"
DL=$(( $(date +%s) + 300 ))
while :; do
    ok=1
    log_has "$c" 'formation_follower_node就绪' || ok=0
    docker exec "$SIMWORLD" grep -q "Ready for takeoff" "/tmp/px4_$NS.log" 2>/dev/null || ok=0
    [ "$ok" = "1" ] && break
    [ "$(date +%s)" -ge "$DL" ] && die "300 秒内没等到 $NS 就绪。看 docker compose logs 排查"
    sleep 5
done
log "$NS 就绪"

source "$ROOT/scripts/contestant_network.sh"
contestant_net_args sim "$ROOT" "$HOME/.cache/contest_sdk" || die "仿真网络参数生成失败"
log "网络：$CONTESTANT_NET_DESC"

# ---- 监视（单机也开：任何飞行测试都要同时起监视，2026-09-30 要求）----
# ⚠️ 下面 --leader 和 --follower 传的是**同一个** $NS。单机测试只有一架飞机，
# 而 monitor.py 要两个命名空间（它本来是给双机编队用的）。后果：**报告图里的
# "编队间距"那条曲线恒为 0，没有意义** ——单机测试要看的是轨迹和高度，不是间距。
# 双机那条路（run.sh / run_real.sh）传的是 NX01/NX02 两个不同的，间距才是真的。
# monitor.py 没有单机模式，这里不改它（它在飞行时跑着，且有未提交改动）。
MON_OUT="/logs/${BASENAME%.py}_sim_report.png"
log "启动监视窗口（报告存 runtime_logs/$(basename "$MON_OUT")）"
docker run -d --name "$C_MONITOR" --network host \
    -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -v "$ROOT/scripts:/scripts:ro" -v "$LOGDIR:/logs" \
    -v "$LAYOUT_HOST:$LAYOUT:ro" \
    "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    python3 -u /scripts/monitor.py --layout "$LAYOUT" \
        --leader "$NS" --follower "$NS" --out "$MON_OUT" >/dev/null 2>&1 \
    || { echo "!! 监视窗口没起来 —— 查 DISPLAY 和 xhost 授权" >&2
         echo "   不中止飞行，但这一轮没有报告图" >&2; }

# ---- 跑测试 ----
# PYTHONPATH 必须在镜像原值后面追加，不能直接覆盖（镜像 ENV 里带着
# /opt/quadrotor_msgs_install/...，直接 -e 覆盖会把它整个丢掉）。
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

log "测试中……（实时日志：docker logs -f $C_TEST）"
DL=$(( $(date +%s) + TIMEOUT_S ))
while docker ps --format '{{.Names}}' | grep -qx "$C_TEST"; do
    if [ "$(date +%s)" -ge "$DL" ]; then
        echo "!! ${TIMEOUT_S} 秒超时，掐掉测试程序" >&2
        docker rm -f "$C_TEST" >/dev/null 2>&1 || true
        break
    fi
    sleep 5
done
docker exec "$C_MONITOR" pkill -INT -f monitor.py >/dev/null 2>&1 || true
# 容器一会儿就被 trap 删掉，`docker logs` 的内容也跟着没了。先把**完整**日志
# 和监视输出落盘——下面只打 tail -25，光靠它排查不了（2026-10-03 实测：
# 第 3 趟规划失败，但前两趟的日志已经被 tail 截掉、无从对比）。
docker logs "$C_TEST"    > "$LOGDIR/${BASENAME%.py}_sim_test.log"    2>&1 || true
docker logs "$C_MONITOR" > "$LOGDIR/${BASENAME%.py}_sim_monitor.log" 2>&1 || true

sleep 3
docker run --rm -v "$LOGDIR:/logs" --entrypoint chown "$IMAGE" \
    -R "$(id -u):$(id -g)" /logs >/dev/null 2>&1 || true

echo "================ $BASENAME 结果（仿真）================"
tail -25 "$LOGDIR/${BASENAME%.py}_sim_test.log"
echo "======================================================="
echo "报告图：runtime_logs/$(basename "$MON_OUT")"
echo "完整日志：runtime_logs/${BASENAME%.py}_sim_test.log"
echo "监视日志：runtime_logs/${BASENAME%.py}_sim_monitor.log"
echo "跑真机同一个测试： cd ../contestant_real && ./run_test.sh $WHICH"
