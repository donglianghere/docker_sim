#!/usr/bin/env bash
# 一键真机：在地面站跑选手程序控制两架真飞机。
#
# 参数就是**要跑的程序文件名**（本目录里的 .py，.py 可省略，可 Tab 补全）：
#
#   ./run_real.sh formation_lite      # 编队飞行
#   ./run_real.sh groundfire_lite     # 地面火情
#   ./run_real.sh highrise_lite       # 高层火情
#   ./run_real.sh mission_lite        # 三轮连贯（火情需**人工布置**，真机没有出题裁判）
#
#   ./run_real.sh                     # 不给就跑 formation_lite.py
#   ./run_real.sh highrise_lite --keep # 结束后保留选手容器，便于翻日志
#
# 跟仿真的 run.sh 的区别（为什么不能直接改域号复用那一个）：
#   · 不起仿真容器。机载飞行栈/视觉栈跑在两架飞机上，由各机的 control_server
#     管理；本脚本只做**探测**，不替你启动，也不在收尾时把它们拆掉。
#   · 就绪判据不同。仿真等 Gazebo 日志里的 "Ready for takeoff"；真机等
#     两机的 ROS 节点可见 + 检测话题有发布者。
#   · 网络参数走 scripts/contestant_network.sh 的 real 模式（ROS_DOMAIN_ID=20
#     + 生成带静态 Peer 的 cyclonedds_real.xml），不是镜像自带的回环配置。
#   · 监视窗口在地面站另起一个容器跑。仿真那边是 docker exec 进本机的
#     flight-stack 容器，真机的 flight-stack 在飞机上，进不去也不该进。
#   · 没有出题裁判。referee.py 靠增删 Gazebo 模型让火情随机出现，真机没有这个
#     手段——综合任务的火情按赛前约定**人工布置**。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

SCRIPT=formation_lite.py
SPACING=4.0
KEEP=0
TIMEOUT_S=1500
LEADER=NX01
FOLLOWER=NX02
LEADER_IP=192.168.2.101
FOLLOWER_IP=192.168.2.102
IMAGE=contestant-sdk:latest
LOGDIR="$ROOT/runtime_logs"
C_LEADER=real_leader
C_FOLLOWER=real_follower
C_MONITOR=real_monitor

while [ $# -gt 0 ]; do
    case "$1" in
        --spacing) SPACING="$2"; shift 2 ;;
        --keep)    KEEP=1; shift ;;
        --timeout) TIMEOUT_S="$2"; shift 2 ;;
        -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
        -*) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
        *)  SCRIPT="${1%.py}.py"; shift ;;
    esac
done

log() { echo "[$(date +%H:%M:%S)] $*"; }
die() { echo "!! $*" >&2; exit 1; }

# 前一次的残余：容器名是固定的，上次若用了 --keep、或者崩了/被 Ctrl-C 掉，
# 容器会留着，这次 docker run --name 会直接报 "name already in use" 起不来。
# 所以**启动前先清一遍**，并注册 trap 保证 Ctrl-C 也清（跟仿真 run.sh 同样的两层）。
cleanup() { [ "$KEEP" = "1" ] && return 0
            docker rm -f "$C_LEADER" "$C_FOLLOWER" "$C_MONITOR" >/dev/null 2>&1 || true; }
trap cleanup EXIT
purge_stale() {
    local n stale=""
    for n in "$C_LEADER" "$C_FOLLOWER" "$C_MONITOR"; do
        docker ps -a --format '{{.Names}}' | grep -qx "$n" && stale="$stale $n"
    done
    [ -z "$stale" ] && return 0
    log "清理上一次残留的容器：$stale"
    docker rm -f $stale >/dev/null 2>&1 || true
}

# 同一时刻只允许一个 run_real.sh：两个并行会互相拆对方的选手容器。
LOCK=/tmp/docker_sim_run_real.lock
exec 9>"$LOCK"
flock -n 9 || die "已经有一个 run_real.sh 在跑（锁 $LOCK）。先等它结束或 ./stop.sh"

# ---- 1. 找程序 ----
SRCDIR="$HERE"
[ -f "$HERE/$SCRIPT" ] || die "找不到程序 $SCRIPT（本目录下可选：$(cd "$HERE" && ls *_lite.py 2>/dev/null | tr '\n' ' '))"
log "程序：$SRCDIR/$SCRIPT"

purge_stale

# ---- 1.8 程序代码一致性 ----
# 真机与仿真的选手程序**必须逐字节相同**，差异只允许出现在 venue.py（场地参数）。
# 这样改场地坐标不会把程序改出分叉，修 bug 也只需要改一边再同步。
# 漂移了就拦下——不然两套代码会悄悄长歪，到现场才发现就晚了。
SIMDIR="$ROOT/contestant_sim"
if [ -d "$SIMDIR" ]; then
    drift=""
    for f in "$HERE"/*_lite.py; do
        b="$(basename "$f")"
        [ -f "$SIMDIR/$b" ] || continue
        cmp -s "$f" "$SIMDIR/$b" || drift="$drift $b"
    done
    if [ -n "$drift" ]; then
        echo "!! 这些程序跟 contestant_sim/ 下的不一致：$drift" >&2
        echo "   场地差异应该只在 venue.py 里。看看差了什么：" >&2
        for b in $drift; do echo "     diff -u $SIMDIR/$b $HERE/$b" >&2; done
        die "程序代码已漂移，先对齐再飞"
    fi
    log "程序代码与仿真版一致（差异只在 venue.py）"
fi

# ---- 2. 仿真残留检查 ----
# 仿真和真机主要靠 ROS_DOMAIN_ID 隔离（21 / 20），话题不会串；但声光常驻程序
# 只有一个容器、只能在一个域，而且 Gazebo 很重会挤占 CPU。这两类隔离不了，
# 必须在起飞前拦住。
"$ROOT/scripts/check_env.sh" real || die "环境检查未通过（见上），处理后再跑"

# ---- 3. 两机可达性与机载栈就绪 ----
# 本脚本只检查机载栈、不代起（机载栈归各机 control_server 管，生命周期比一次
# 飞行长，重起还要重锁原点）。但报错时要把补救命令打出来，跟下面视觉那步一致，
# 不然人得翻手册才知道敲什么。
hint_stack() {   # hint_stack <ip> <flight|vision>
    echo "   起它：地面站界面点按钮，或" >&2
    echo "     curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \\" >&2
    echo "          -d '{\"stack\":\"$2\",\"action\":\"up\",\"confirm\":true}' \\" >&2
    echo "          http://$1:8890/stack" >&2
}

for ip in "$LEADER_IP" "$FOLLOWER_IP"; do
    ping -c1 -W2 "$ip" >/dev/null 2>&1 || die "飞机 $ip 不可达（检查 WiFi 和飞机电源）"
done
log "两机网络可达"

for ip in "$LEADER_IP" "$FOLLOWER_IP"; do
    st=$(curl -s --noproxy '*' --max-time 8 "http://$ip:8890/status" 2>/dev/null || true)
    if [ -z "$st" ]; then
        echo "!! 飞机 $ip 的 control_server(8890) 没响应" >&2
        echo "   它是开机自启的，没响应说明服务本身挂了（不是容器没起）。上机查：" >&2
        echo "     ssh nvidia@$ip 'systemctl status uav-control-server --no-pager -l | head -20'" >&2
        echo "     ssh nvidia@$ip 'sudo systemctl restart uav-control-server'" >&2
        die "机载管控服务不可用，已中止"
    fi
    # /status 直接给 flight_up / vision_up / namespace，比"端口有响应"硬得多
    ns=$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("namespace",""))' 2>/dev/null)
    fu=$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("flight_up"))' 2>/dev/null)
    vu=$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("vision_up"))' 2>/dev/null)
    if [ "$fu" != "True" ]; then
        echo "!! 飞机 $ip 的 flight-stack 没起（/status: flight_up=$fu）" >&2
        hint_stack "$ip" flight
        die "机载飞行栈未就绪，已中止"
    fi
    if [ "$vu" != "True" ]; then
        echo "!! 飞机 $ip 的 vision-stack 没起（/status: vision_up=$vu）" >&2
        hint_stack "$ip" vision
        echo "   注意：容器起来之后检测节点还没跑，再执行 ./vision_real.sh up" >&2
        die "机载视觉栈未就绪，已中止"
    fi
    log "  $ip: namespace=$ns  flight_up=$fu  vision_up=$vu"
done
# 命名空间别装错机：两机 .namespace 配反过的话，话题全错位且不报错
ns1=$(curl -s --noproxy '*' --max-time 8 "http://$LEADER_IP:8890/status" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("namespace",""))' 2>/dev/null || true)
ns2=$(curl -s --noproxy '*' --max-time 8 "http://$FOLLOWER_IP:8890/status" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("namespace",""))' 2>/dev/null || true)
[ "$ns1" = "$LEADER" ] || die "长机 IP $LEADER_IP 报的命名空间是 $ns1，期望 $LEADER（.namespace 配反了？）"
[ "$ns2" = "$FOLLOWER" ] || die "僚机 IP $FOLLOWER_IP 报的命名空间是 $ns2，期望 $FOLLOWER"
log "两机机载栈就绪，命名空间对应正确"

# ---- 4. 网络参数（真机模式）----
# shellcheck source=/dev/null
source "$ROOT/scripts/contestant_network.sh"
contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" || die "真机网络参数生成失败"
log "网络：$CONTESTANT_NET_DESC"

# ---- 5. 视觉就绪判据（真机特有）----
# 机载的检测节点**不随栈自启**，要靠 POST /vision/mode 拉起，而且不持久化。
# 不检查的话会出现"飞行栈一切正常、wait_for_detection 永远等不到"的静默失效。
log "检查视觉就绪（四条检测话题都要有发布者）"
ready=$(timeout 90 docker run --rm --network host "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null
              # 轮询而不是死等：DDS 发现通常几秒收敛，写死 sleep 25 等于每次都
              # 付最坏情况的代价。一发现够数就立刻返回，最多等约 30 秒。
              n=0; for i in $(seq 1 15); do
                  n=$(timeout 6 ros2 topic list --no-daemon 2>/dev/null | grep -c "vision/detections")
                  [ "$n" -ge 2 ] && break
                  sleep 2
              done; echo "$n"' 2>/dev/null \
         | tr -d '\r' | grep -E '^[0-9]+$' | tail -1 || echo 0)
# 注意 tail/grep：镜像 entrypoint 会往 stdout 打一行横幅，不过滤的话会连横幅一起
# 赋给 ready，后面的整数比较就报 "需要整数表达式"。
if [ "${ready:-0}" -lt 2 ]; then
    echo "!! 只发现 ${ready:-0} 条 vision/detections（应为 2：NX01 + NX02）" >&2
    echo "   机载检测节点没起。对每架飞机执行：" >&2
    for ip in "$LEADER_IP" "$FOLLOWER_IP"; do
        echo "     curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \\" >&2
        echo "          -d '{\"cam\":\"cam0\",\"mode\":\"yolo\"}' http://$ip:8890/vision/mode" >&2
        echo "     （cam1 同理，两路都要起）" >&2
    done
    echo "   四条一起起的捷径： ./vision_real.sh up && ./vision_real.sh status" >&2
    die "视觉未就绪，已中止（这一步不拦住的话，火情相关动作会静默超时）"
fi
log "视觉就绪（$ready 条检测话题）"

# ---- 6. 声光（只接地面站）----
sl_dom=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' contestant-sound-light 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
if [ -z "$sl_dom" ]; then
    echo "（提示：声光常驻程序没在跑，播报不会出声。要开：./start_sound_light_server.sh --real /dev/ttyUSB0）" >&2
elif [ "$sl_dom" != "20" ]; then
    echo "!! 声光容器在 ROS_DOMAIN_ID=$sl_dom（仿真域），真机播报收不到 !!" >&2
    echo "   重起：./start_sound_light_server.sh --real /dev/ttyUSB0" >&2
fi

# ---- 7. 监视窗口 ----
# 必开：任何飞行测试都要同时起编队监视。数据源是两机的 uwb/pose_abs。
MON_OUT="/logs/${SCRIPT%.py}_real_formation.png"
MON_LOG="/logs/${SCRIPT%.py}_real_monitor.log"
log "启动监视窗口（报告将存到 runtime_logs/$(basename "$MON_OUT")）"
docker run -d --name "$C_MONITOR" --network host \
    -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -v "$ROOT/scripts:/scripts:ro" -v "$LOGDIR:/logs" \
    "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    python3 -u /scripts/monitor.py --leader "$LEADER" --follower "$FOLLOWER" \
        --spacing "$SPACING" --out "$MON_OUT" >/dev/null 2>&1 \
    || echo "（监视没起来，不影响飞行）" >&2

# ---- 8. 两个选手程序 ----
BASE_PYPATH="$(docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$IMAGE" \
               | sed -n 's/^PYTHONPATH=//p' | head -1)"
PYPATH="/workspace${BASE_PYPATH:+:$BASE_PYPATH}"; PYPATH="${PYPATH%:}"
COMMON=(--network host
        -v /etc/localtime:/etc/localtime:ro
        -v "$SRCDIR:/workspace:ro"
        -v "$LOGDIR:/logs"
        -e PYTHONPATH="$PYPATH"
        "${CONTESTANT_NET_ARGS[@]}")

log "启动选手程序：$SCRIPT（长机=$LEADER 僚机=$FOLLOWER 间距=${SPACING}米）"
docker run -d --name "$C_LEADER" "${COMMON[@]}" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$LEADER" --role leader \
    --teammate "$FOLLOWER" --spacing "$SPACING" >/dev/null
docker run -d --name "$C_FOLLOWER" "${COMMON[@]}" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$FOLLOWER" --role follower \
    --teammate "$LEADER" --spacing "$SPACING" >/dev/null

echo
log "飞行中……（实时日志：docker logs -f $C_LEADER）"
DL=$(( $(date +%s) + TIMEOUT_S ))
while :; do
    live=0
    for c in "$C_LEADER" "$C_FOLLOWER"; do
        docker ps --format '{{.Names}}' | grep -qx "$c" && live=$((live+1))
    done
    [ "$live" = "0" ] && break
    if [ "$(date +%s)" -ge "$DL" ]; then
        echo "!! 超过 ${TIMEOUT_S} 秒仍未结束 !!" >&2
        for c in "$C_LEADER" "$C_FOLLOWER"; do echo "--- $c:" >&2; docker logs --tail 8 "$c" >&2 2>&1; done
        exit 1
    fi
    sleep 5
done

# ---- 9. 收尾（只收地面站这边，机载栈不动）----
docker exec "$C_MONITOR" pkill -INT -f monitor.py >/dev/null 2>&1 || true
sleep 6
docker run --rm -v "$LOGDIR:/logs" --entrypoint chown "$IMAGE" \
    -R "$(id -u):$(id -g)" /logs >/dev/null 2>&1 || true

echo
echo "================ $SCRIPT 结果 ================"
rc=0
for c in "$C_LEADER" "$C_FOLLOWER"; do
    code="$(docker inspect -f '{{.State.ExitCode}}' "$c" 2>/dev/null || echo '?')"
    echo "--- $c 退出码 $code"
    [ "$code" = "0" ] || rc=1
    docker logs "$c" 2>&1 | grep -E 'Traceback|Error|错误|失败' | tail -5 || true
done
echo
echo "报告图：runtime_logs/$(basename "$MON_OUT")"
echo "机载栈仍在运行（本脚本不拆机载容器）。要停机载栈走地面站网页或各机的 control_server。"

if [ "$KEEP" = "0" ]; then
    docker rm -f "$C_LEADER" "$C_FOLLOWER" "$C_MONITOR" >/dev/null 2>&1 || true
else
    echo "（--keep：选手容器已保留，翻完日志后 docker rm -f $C_LEADER $C_FOLLOWER $C_MONITOR）"
fi
exit $rc
