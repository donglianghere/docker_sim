#!/usr/bin/env bash
# 一键跑单机测试。跟 run_real.sh 共用同一套前置闸门，但**只起一架**。
#
#   ./run_test_real.sh t1              # 物资抓取（默认 NX02，带夹爪）
#   ./run_test_real.sh t2              # 地面火情对准（默认 NX01）
#   ./run_test_real.sh t3 --n          # 高层火情瞄准，用 N 点看 1# 楼
#   ./run_test_real.sh t4 --nx02       # 避障，指定用 NX02
#   ./run_test_real.sh t5 --keep       # 仿地，结束后保留容器看日志
#   ./run_test_real.sh                 # 不给就列出可选的测试
#
# 默认机号是按测试内容定的：t1 要夹爪所以默认 NX02（任务机），其余默认 NX01。
# 用 --nx01 / --nx02 可以覆盖。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TDIR="$HERE/单机测试_real"
# 测试程序 import venue，而 venue.py 在选手目录——所以选手目录挂 /workspace
# （PYTHONPATH 第一条），测试程序自己挂 /tests。
VENUEDIR="$ROOT/contestant_real"
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
    echo "用法： ./run_test_real.sh t1 [--nx01|--nx02] [--keep] [--timeout 秒] [程序自己的参数...]"
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
# 角色按机号定，别写死（2026-10-16）：SDK 把 --role leader 映射成 role='recon'，
# 而 takeoff()/land() 的**自动**声光播报前缀就取自它（recon=侦察机 / supply=任务机，
# 见 capabilities.py 的 _SOUND_LIGHT_ROLE_PREFIX）。原来这里固定传 leader，于是在
# NX02 上跑单机测试会播"侦察机起飞/侦察机降落"——程序里显式给了 sound= 的那几条
# （"任务机抓取灭火弹"等）倒是对的，只有自动播报错，所以一直没人注意到。
case "$NS" in NX01) ROLE=leader ;; NX02) ROLE=follower ;; *) ROLE=leader ;; esac

# ---- 程序自己声明要不要飞机（2026-10-04）----
# 有的测试压根不碰飞机（比如 t6 只测地面站上那块声光板），那些前置检查会把它
# 卡死在"飞机不可达"上。**由程序声明、不由运行器按文件名猜**——文件名一改就
# 失效，新测试还得回来改运行器的表，这个教训见 run_real.sh 里"相机不在这里配"
# 那段。程序里写 `NEEDS_AIRCRAFT = False` 即可。
NEEDS_AIRCRAFT=1
grep -qE '^NEEDS_AIRCRAFT[[:space:]]*=[[:space:]]*False' "$SCRIPT" && NEEDS_AIRCRAFT=0
if [ "$NEEDS_AIRCRAFT" = 1 ]; then
    log "测试：$BASENAME    飞机：$NS ($IP)"
else
    log "测试：$BASENAME    **不连飞机**（本测试只用地面站设备）"
    log "  供参考：飞机 IP 是 NX01=$NX01_IP / NX02=$NX02_IP，本测试不需要它们在线"
fi

# ---- 互斥：跟 run_real.sh 共用一把锁，不能同时飞 ----
LOCK=/tmp/docker_sim_run_real.lock
exec 9>"$LOCK"
flock -n 9 || die "已经有 run_real.sh 或另一个 run_test.sh 在跑"

# ---- 前置闸门（跟 run_real.sh 同一套，只查这一架）----
purge_stale
# ---- 共享栈必起（2026-10-03 用户要求，同 run_real.sh）----
source "$ROOT/scripts/ensure_stacks.sh"
ensure_sound_light real
ensure_gcs_web

"$ROOT/scripts/check_env.sh" real || die "环境检查未通过"

if [ "$NEEDS_AIRCRAFT" = 1 ]; then
ping -c1 -W2 "$IP" >/dev/null 2>&1 || die "飞机 $IP 不可达"
# 只检查机载栈、不代起（理由同 run_real.sh）。报错时把补救命令打出来。
hint_stack() {   # hint_stack <ip> <flight|vision>
    echo "   起它：地面站界面点按钮，或" >&2
    echo "     curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \\" >&2
    echo "          -d '{\"stack\":\"$2\",\"action\":\"up\",\"confirm\":true}' \\" >&2
    echo "          http://$1:8890/stack" >&2
}

# `|| true` 不能省：set -e 下 curl 连不上(退出码7)会让这行赋值失败，脚本带着
# 7 直接退出，下面那条"control_server 没响应"的提示根本到不了。
st="$(curl -s --noproxy '*' --max-time 8 "http://$IP:8890/status" 2>/dev/null || true)"
if [ -z "$st" ]; then
    echo "!! 飞机 $IP 的 control_server(8890) 没响应" >&2
    echo "   它是开机自启的，没响应说明服务本身挂了（不是容器没起）。上机查：" >&2
    echo "     ssh nvidia@$IP 'systemctl status uav-control-server --no-pager -l | head -20'" >&2
    echo "     ssh nvidia@$IP 'sudo systemctl restart uav-control-server'" >&2
    die "机载管控服务不可用，已中止"
fi
ns_rep="$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("namespace",""))' 2>/dev/null)"
fu="$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("flight_up"))' 2>/dev/null)"
vu="$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("vision_up"))' 2>/dev/null)"
[ "$ns_rep" = "$NS" ] || die "$IP 报的命名空间是 $ns_rep，期望 $NS"
if [ "$fu" != "True" ]; then
    echo "!! $IP 的 flight-stack 没起（/status: flight_up=$fu）" >&2
    hint_stack "$IP" flight
    die "机载飞行栈未就绪，已中止"
fi
if [ "$vu" != "True" ]; then
    echo "!! $IP 的 vision-stack 没起（/status: vision_up=$vu）" >&2
    hint_stack "$IP" vision
    echo "   注意：容器起来之后检测节点还没跑，再执行 ./vision_real.sh up" >&2
    die "机载视觉栈未就绪，已中止"
fi
log "机载栈就绪：$ns_rep  flight_up=$fu  vision_up=$vu"
fi

# 相机：不在这里配，由程序自己声明（见 run_real.sh 同名处的说明）。
# 但网络参数下面起容器要用，这里先算出来。
# shellcheck source=/dev/null
source "$ROOT/scripts/contestant_network.sh"
# 不连飞机的测试用 local（域 20 + 仅回环）：它只跟本机的声光常驻程序说话，
# 不该因为"地面站没连飞机那个 WiFi"就跑不了——2026-10-04 用户要求。
_NET_MODE=real
[ "$NEEDS_AIRCRAFT" = 1 ] || _NET_MODE=local
contestant_net_args "$_NET_MODE" "$ROOT" "$HOME/.cache/contest_sdk" || die "网络参数生成失败"
# t4/t5 跳过了上面的 source，这里补上
[ ${#CONTESTANT_NET_ARGS[@]} -gt 0 ] 2>/dev/null || {
    source "$ROOT/scripts/contestant_network.sh"
    contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" || die "网络参数生成失败"
}
log "网络：$CONTESTANT_NET_DESC"

# ---- 机载 MAVLink 流收窄（降 CPU，2026-10-03）----
# 飞控 SD 卡把 ODOMETRY / LOCAL_POSITION_NED / ATTITUDE_QUATERNION /
# HIGHRES_IMU 四路顶到了 100Hz，实测白占机载整机 8-9 个百分点的 CPU。改那四
# 行要拆机取 SD 卡，所以在这里用 MAV_CMD_SET_MESSAGE_INTERVAL(511) 运行时压
# 回来。每一路关/降的依据和完整实测数据见 scripts/tune_mavlink_streams.sh
# 文件头。失败只警告不中止（不收窄也能飞，只是 CPU 高一截）。
# 必须放在 contestant_net_args 之后——它要靠 CONTESTANT_NET_ARGS 进真机域。
if [ "$NEEDS_AIRCRAFT" = 1 ]; then
    source "$ROOT/scripts/tune_mavlink_streams.sh"
    log "收窄机载 MAVLink 高频流（降 CPU）"
    tune_mavlink_streams "$NS"
fi

# ---- 监视（单机也开：高度曲线是 t5 的主要判据）----
MON_OUT=""
if [ "$NEEDS_AIRCRAFT" = 1 ]; then
MON_OUT="/logs/${BASENAME%.py}_report.png"
log "启动监视窗口（报告存 runtime_logs/$(basename "$MON_OUT")）"
# ⚠️ 下面 --leader 和 --follower 传的是**同一个** $NS。单机测试只有一架飞机，
# 而 monitor.py 要两个命名空间（它本来是给双机编队用的）。后果：报告图里的
# "编队间距"那条曲线**恒为 0、没有意义**——单机测试看的是轨迹和高度。
# 双机那条路（run_real.sh）传 NX01/NX02 两个不同的，间距才是真的。
# 俯视底图：monitor.py 不给 --layout 就退回写死的 fire_drill_room 老坐标
# （房间原点在**中心**、立柱 (4.5,7)），跟样题场景完全不是一套，轨迹会叠在
# 错误的底图上。样题布局的权威来源是 src/contest_mission/config 下那份 yaml
# （room/pillars/pads/obstacle_cylinder/terrain_module 都在里面）。
# 2026-10-05 补：之前一直没传，每趟监视器都在打那条"没给 --layout"的警告。
LAYOUT_YAML="${LAYOUT_YAML:-sample_room_layout.yaml}"
LAYOUT_HOST="$ROOT/src/contest_mission/config/$LAYOUT_YAML"
MON_LAYOUT_ARGS=()
if [ -f "$LAYOUT_HOST" ]; then
    MON_LAYOUT_ARGS=(-v "$(dirname "$LAYOUT_HOST")":/layout:ro)
    MON_LAYOUT_OPT=(--layout "/layout/$LAYOUT_YAML")
else
    echo "!! 找不到布局 $LAYOUT_HOST —— 监视器底图会退回老坐标" >&2
    MON_LAYOUT_OPT=()
fi
docker run -d --name "$C_MONITOR" --network host \
    -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -v "$ROOT/scripts:/scripts:ro" -v "$LOGDIR:/logs" \
    "${MON_LAYOUT_ARGS[@]}" \
    "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    python3 -u /scripts/monitor.py --leader "$NS" --follower "$NS" \
        "${MON_LAYOUT_OPT[@]}" --out "$MON_OUT" >/dev/null 2>&1 \
    || { echo "!! 监视窗口没起来 —— 查 DISPLAY 和 xhost 授权" >&2
       echo "   不中止飞行，但这一轮没有编队间距记录和报告图" >&2; }
else
    # 不连飞机的测试没有轨迹可画，监视窗口没有意义。注意
    # feedback"任何飞行测试都要同时起监视窗口"针对的是**飞行**测试，这里不是。
    log "不起监视窗口（本测试不飞，没有轨迹）"
fi

# ---- 跑 ----
BASE_PYPATH="$(docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$IMAGE" \
               | sed -n 's/^PYTHONPATH=//p' | head -1)"
PYPATH="/workspace${BASE_PYPATH:+:$BASE_PYPATH}"; PYPATH="${PYPATH%:}"
log "启动测试程序"
# CONTEST_CONTROL_HOST：见 run_real.sh 同名处的说明。不注入 SDK 会判成仿真。
docker run -d --name "$C_TEST" -e CONTEST_CONTROL_HOST="$IP" --network host \
    -v /etc/localtime:/etc/localtime:ro \
    -v "$VENUEDIR:/workspace:ro" -v "$TDIR:/tests:ro" -v "$LOGDIR:/logs" \
    -e PYTHONPATH="$PYPATH" "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    python3 -u "/tests/$BASENAME" \
      --namespace "$NS" --role "$ROLE" --teammate "$NS" "${EXTRA[@]}" >/dev/null

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

# 日志必须**先落盘再清容器**。2026-10-04 修：原来这里直接 grep
# "$LOGDIR/..._test.log"，可是全脚本没有任何一步生成那个文件——于是错误摘要
# 永远是空的，紧接着下面 docker rm -f 把容器删掉，异常原文彻底丢失。t4 那次
# "飞完3趟一直悬停、退出码1"就是因为这个查不下去。
docker logs "$C_TEST" > "$LOGDIR/${BASENAME%.py}_test.log" 2>&1 || true
docker logs "$C_MONITOR" > "$LOGDIR/${BASENAME%.py}_monitor.log" 2>&1 || true

echo
echo "================ $BASENAME 结果 ================"
code="$(docker inspect -f '{{.State.ExitCode}}' "$C_TEST" 2>/dev/null || echo '?')"
echo "退出码 $code"
grep -E 'Traceback|Error|错误|失败|!!' "$LOGDIR/${BASENAME%.py}_test.log" | tail -8 || true
echo
[ -n "$MON_OUT" ] && echo "报告图：runtime_logs/$(basename "$MON_OUT")"
echo "完整日志：runtime_logs/${BASENAME%.py}_test.log"
echo "监视日志：runtime_logs/${BASENAME%.py}_monitor.log"
echo "机载栈仍在运行（本脚本不拆机载容器）。"

[ "$KEEP" = "0" ] && docker rm -f "$C_TEST" "$C_MONITOR" >/dev/null 2>&1 || \
    echo "（--keep：容器已保留，看完 docker rm -f $C_TEST $C_MONITOR）"
exit "${code:-1}"
