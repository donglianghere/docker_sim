#!/usr/bin/env bash
# 一键真机：在地面站跑选手程序，控制两架真飞机。
#
# 用法：./run_real.sh [程序名] [选项]   程序名 = 本目录下的 .py（.py 可省、可 Tab 补全）
#   ./run_real.sh formation_slim       编队飞行（不给程序名时的默认）
#   ./run_real.sh groundfire_slim      地面火情
#   ./run_real.sh highrise_slim        高层火情
#   ./run_real.sh mission_slim         三轮连贯（火情要人工布置，真机没有出题裁判）
#
# 选项：
#   --spacing N          编队目标间距（米），跟 --min-sep 一起调
#   --min-sep N          两机直线距离下限（米），打到飞机上，只对本次飞行有效
#   --max-vel N          经规划器航段的巡航限速（m/s）
#   --skip-clock-check   跳过两机时钟同步闸门
#   --allow-drift        程序代码与仿真不一致时只告警不拦（现场应急，会留痕）
#   --sim-frozen         仿真侧已冻结：跳过"四个流程程序两侧逐字节相同"这一项，
#                        赛场任务逻辑可以独立演进。另两项照查（见下）。
#   --keep               结束后保留选手容器，便于翻日志
#
# 跟仿真 run.sh 的区别：不起飞行栈（它在飞机上，本脚本只探测）、就绪判据是两机
# ROS 节点可见 + 检测话题有发布者、网络走 real 模式、监视窗口另起容器、没有裁判。
#
# 环境变量：SOUND_LIGHT_IGNORE=1 跳过声光串口检查（本轮没有声光播报）。
#           SIM_FROZEN=1 等同于 --sim-frozen。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

SCRIPT=formation_slim.py   # 2026-10-14：原 formation_lite.py 已删
# 编队目标间距（米）：传给选手程序覆盖 SDK 默认值。
SPACING=5.0
# 两机直线距离下限（米），低于它僚机参考点冻结。留空=用飞机上的现值；调 --spacing 要一起调。
MIN_SEP=
KEEP=0
TIMEOUT_S=1500
SKIP_CLOCK=0
# 经规划器航段的巡航限速（m/s）。不管直飞。
MAX_VEL=0.6   # 主控：会覆盖 .env 的 LINE_CRUISE_MPS
LEADER=NX01
FOLLOWER=NX02
LEADER_IP=192.168.2.101
FOLLOWER_IP=192.168.2.102
IMAGE=contestant-sdk:latest
LOGDIR="$ROOT/runtime_logs"
C_LEADER=real_leader
C_FOLLOWER=real_follower
C_MONITOR=real_monitor
ALLOW_DRIFT=0
# 仿真侧冻结开关：1 = 不再要求流程程序两侧逐字节相同。环境变量可预置。
SIM_FROZEN=${SIM_FROZEN:-0}

while [ $# -gt 0 ]; do
    case "$1" in
        --spacing) SPACING="$2"; shift 2 ;;
        --min-sep) MIN_SEP="$2"; shift 2 ;;
        --keep)    KEEP=1; shift ;;
        --skip-clock-check) SKIP_CLOCK=1; shift ;;
        --allow-drift) ALLOW_DRIFT=1; shift ;;
        --sim-frozen) SIM_FROZEN=1; shift ;;
        --max-vel) MAX_VEL="$2"; shift 2 ;;
        --timeout) TIMEOUT_S="$2"; shift 2 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        -*) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
        *)  SCRIPT="${1%.py}.py"; shift ;;
    esac
done

log() { echo "[$(date +%H:%M:%S)] $*"; }
die() { echo "!! $*" >&2; exit 1; }

# 容器名固定：上一趟用了 --keep 或崩了/被 Ctrl-C，残留要先清掉。
RUN_STAMP="$(date +%Y%m%d-%H%M%S)"

# 把这一趟的日志和监视器产物归档到带时间戳的目录。
archive_run() {
    local dir="$LOGDIR/archive/${SCRIPT%.py}_$RUN_STAMP"
    mkdir -p "$dir" 2>/dev/null || return 0
    local c
    for c in "$C_LEADER" "$C_FOLLOWER" "$C_MONITOR"; do
        docker logs "$c" > "$dir/$c.log" 2>&1 || true
        # 容器不存在时 docker logs 会把报错写进文件，空文件删掉。
        [ -s "$dir/$c.log" ] || rm -f "$dir/$c.log"
        grep -q "No such container" "$dir/$c.log" 2>/dev/null && rm -f "$dir/$c.log"
    done
    # 监视器的 CSV/PNG 每趟同名覆盖，这里拷一份进归档。
    local f
    for f in "${SCRIPT%.py}_real_formation.csv" "${SCRIPT%.py}_real_formation_diag.csv" \
             "${SCRIPT%.py}_real_formation.png"; do
        [ -f "$LOGDIR/$f" ] && cp -p "$LOGDIR/$f" "$dir/" 2>/dev/null
    done
    if [ -n "$(ls -A "$dir" 2>/dev/null)" ]; then
        echo "[$(date +%H:%M:%S)] 本趟归档：runtime_logs/archive/${SCRIPT%.py}_$RUN_STAMP" >&2
    else
        rmdir "$dir" 2>/dev/null || true
    fi
}

cleanup() { archive_run
            [ "$KEEP" = "1" ] && return 0
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

# 同一时刻只允许一个 run_real.sh：并行会互相拆对方的选手容器。
LOCK=/tmp/docker_sim_run_real.lock
exec 9>"$LOCK"
flock -n 9 || die "已经有一个 run_real.sh 在跑（锁 $LOCK）。先等它结束或 ./stop.sh"

# ---- 1. 找程序 ----
SRCDIR="$HERE"
# 列举时排除 venue.py（坐标表，不是可跑的程序）。
[ -f "$HERE/$SCRIPT" ] || die "找不到程序 $SCRIPT（本目录下可选：$(cd "$HERE" && ls *.py 2>/dev/null | grep -v '^venue\|^rounds\.py$' | tr '\n' ' '))"
log "程序：$SRCDIR/$SCRIPT"

purge_stale

# ---- 1.8 程序代码一致性 ----
SIMDIR="$ROOT/contestant_sim"
if [ -d "$SIMDIR" ]; then
    drift=""
    # 2026-10-23 这道闸门拆成三项，只有第一项受 SIM_FROZEN 影响：
    #   ① 流程程序逐字节相同 —— 防"改了真机忘了仿真"。但仿真一旦冻结、不再
    #      跑了，它就变成"要求你同步维护一个不会执行的副本"，赛场临时改任务
    #      逻辑时纯粹是摩擦，所以可以用 --sim-frozen / SIM_FROZEN=1 关掉。
    #   ② rounds.py 两侧逐字节一致 —— **跟冻结无关，永远查**。
    #      2026-10-24 起它从 SDK 镜像搬到了选手目录（挂载进容器，改完不用重建
    #      镜像）。既然成了选手文件，就跟四个任务程序一样要求两侧同步：它是
    #      公共飞行段，真机改了而仿真没改，等于仿真验证的不是同一套流程。
    #   ③ 两份 venue 顶层常量名一致 —— **跟冻结无关，永远查**。rounds.py 两套
    #      场地共用，真机 venue 加了常量而仿真没有，仿真那套就启动即 ImportError；
    #      仿真是交付给高校实验室的产品，赛后还要能跑。
    if [ "$SIM_FROZEN" = "1" ]; then
        log "SIM_FROZEN：跳过流程程序逐字节比对（仿真侧已冻结，赛场逻辑独立演进）"
    else
        # 比两侧同名的所有 .py；真机独有的文件名（赛场特供流程）天然豁免。
        # 单机测试和地面检查 2026-10-21 移到 docker_sim/test/，不在这道闸门里。
        for f in "$HERE"/*.py; do
            [ -e "$f" ] || continue
            b="${f#$HERE/}"                 # 保留相对路径，便于区分同名文件
            if [ "$(basename "$f")" = venue.py ]; then continue; fi
            if [ ! -f "$SIMDIR/$b" ]; then continue; fi
            cmp -s "$f" "$SIMDIR/$b" || drift="$drift $b"
        done
    fi
    # rounds.py 2026-10-24 起住在选手目录（挂载、免重建镜像）。它是两侧共用的
    # 公共飞行段，所以要求逐字节一致；少了一边就是没同步，直接拦。
    if [ ! -f "$HERE/rounds.py" ]; then
        drift="$drift rounds.py(真机侧缺失)"
    elif [ ! -f "$SIMDIR/rounds.py" ]; then
        drift="$drift rounds.py(仿真侧缺失)"
    elif ! cmp -s "$HERE/rounds.py" "$SIMDIR/rounds.py"; then
        drift="$drift rounds.py"
    fi
    # 两份 venue 的顶层常量名必须一致，少一个就是选手程序启动时 ImportError。
    if ! diff -q <(grep -oE '^[A-Z_][A-Z0-9_]*' "$HERE/venue.py" | sort -u) \
                 <(grep -oE '^[A-Z_][A-Z0-9_]*' "$SIMDIR/venue.py" | sort -u) >/dev/null; then
        drift="$drift venue.py(两侧常量名不一致)"
    fi
    if [ -n "$drift" ]; then
        echo "!! 跟 contestant_sim/ 不一致：$drift" >&2
        echo "   场地差异应该只在 venue.py 的值里；流程要分叉请用新文件名。" >&2
        for b in $drift; do
            case "$b" in *'('*) continue ;; esac
            echo "     diff -u $SIMDIR/$b $HERE/$b" >&2
        done
        if [ "$ALLOW_DRIFT" = "1" ]; then
            log "⚠ --allow-drift 放行：本架次飞的不是仿真验证过的那一套（上面是清单）"
        else
            die "程序代码已漂移，先对齐再飞；确要放行用 --allow-drift"
        fi
    elif [ "$SIM_FROZEN" = "1" ]; then
        log "rounds.py 与两侧 venue 常量名均已对齐"
    else
        log "程序代码与仿真版一致（差异只在 venue.py）"
    fi
fi

# ---- 2. 仿真残留检查 ----
# ---- 共享栈必起（2026-10-03 用户要求）----
source "$ROOT/scripts/ensure_stacks.sh"
check_sound_light_port
ensure_sound_light real
ensure_gcs_web

"$ROOT/scripts/check_env.sh" real || die "环境检查未通过（见上），处理后再跑"

# ---- 3. 两机可达性与机载栈就绪 ----
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
    # /status 直接给 flight_up / vision_up / namespace，比"端口有响应"硬得多。
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
# 命名空间别装错机：两机配反的话题全错位，而且不报错。
ns1=$(curl -s --noproxy '*' --max-time 8 "http://$LEADER_IP:8890/status" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("namespace",""))' 2>/dev/null || true)
ns2=$(curl -s --noproxy '*' --max-time 8 "http://$FOLLOWER_IP:8890/status" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("namespace",""))' 2>/dev/null || true)
[ "$ns1" = "$LEADER" ] || die "长机 IP $LEADER_IP 报的命名空间是 $ns1，期望 $LEADER（.namespace 配反了？）"
[ "$ns2" = "$FOLLOWER" ] || die "僚机 IP $FOLLOWER_IP 报的命名空间是 $ns2，期望 $FOLLOWER"
log "两机机载栈就绪，命名空间对应正确"

# ---- 3.5 时钟同步闸门（2026-10-10 加）----
GCS_IP=192.168.2.103
CLOCK_MAX_OFFSET_S=0.05   # 单机 RMS 偏移上限。两机各 0.05 → 相对 0.1，仍远低于 0.25
if [ "$SKIP_CLOCK" = 1 ]; then
    log "△ 跳过时钟同步检查（--skip-clock-check）——编队协同可能失效"
else
    # chrony 的 Reference ID 是选中源 IP 的十六进制，两机必须都选地面站。
    want_hex=$(LC_ALL=C python3 -c \
        'import sys;print("".join("%02X"%int(x) for x in sys.argv[1].split(".")))' "$GCS_IP")
    for ip in "$LEADER_IP" "$FOLLOWER_IP"; do
        # 机载 locale 不把 . 当小数点，两边都要钉 LC_ALL=C。
        trk=$(timeout 8 ssh -o BatchMode=yes -o ConnectTimeout=4 "nvidia@$ip" \
                'LC_ALL=C chronyc -n tracking' 2>/dev/null || true)
        if [ -z "$trk" ]; then
            echo "!! 飞机 $ip 读不到 chrony 状态（ssh 不通，或 chronyd 没在跑）" >&2
            echo "   查：ssh nvidia@$ip 'systemctl status chrony --no-pager | head -5'" >&2
            die "时钟状态未知，已中止（确认无妨可加 --skip-clock-check）"
        fi
        ref=$(echo "$trk" | LC_ALL=C awk -F': *' '/^Reference ID/{print $2}' | awk '{print $1}')
        rms=$(echo "$trk" | LC_ALL=C awk -F': *' '/^RMS offset/{printf "%.6f", $2+0}')
        if [ "$ref" != "$want_hex" ]; then
            echo "!! 飞机 $ip 没有对时到地面站：当前源 $ref，期望 $want_hex（$GCS_IP）" >&2
            echo "$trk" | grep -E "Reference ID|Stratum|RMS offset" | sed 's/^/     /' >&2
            echo "   两机对不同的源 → swarm 轨迹互相丢弃 → 编队全程失效。修它：" >&2
            echo "     ${ROOT%/*}/tools/setup_timesync.sh $ip" >&2
            echo "     ssh nvidia@$ip 'chronyc -n sources'   # 确认 ^* 指向 $GCS_IP" >&2
            die "时钟源不是地面站，已中止"
        fi
        if LC_ALL=C awk -v v="$rms" -v m="$CLOCK_MAX_OFFSET_S" 'BEGIN{exit !(v>m)}'; then
            echo "!! 飞机 $ip 时钟偏移过大：RMS ${rms}s > ${CLOCK_MAX_OFFSET_S}s" >&2
            echo "   源选对了但还没收敛（刚开机？）。等十几秒再看：" >&2
            echo "     ssh nvidia@$ip 'chronyc -n tracking'" >&2
            die "时钟未收敛，已中止"
        fi
        log "  $ip: 对时到 $GCS_IP，RMS 偏移 ${rms}s"
    done
    log "两机时钟源统一（均为地面站 $GCS_IP）"
fi

# ---- 4. 网络参数（真机模式）----
source "$ROOT/scripts/contestant_network.sh"
contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" || die "真机网络参数生成失败"
log "网络：$CONTESTANT_NET_DESC"

# ---- 机载 MAVLink 流收窄（降 CPU，2026-10-03）----
source "$ROOT/scripts/tune_mavlink_streams.sh"
log "收窄机载 MAVLink 高频流（降 CPU）"
tune_mavlink_streams "$LEADER" "$FOLLOWER"


# ---- 6. 声光（只接地面站）----
sl_dom=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' contestant-sound-light 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
if [ -z "$sl_dom" ]; then
    echo "（提示：声光常驻程序没在跑，播报不会出声。要开：./start_sound_light_server.sh --real /dev/ttyUSB0）" >&2
elif [ "$sl_dom" != "20" ]; then
    echo "!! 声光容器在 ROS_DOMAIN_ID=$sl_dom（仿真域），真机播报收不到 !!" >&2
    echo "   重起：./start_sound_light_server.sh --real /dev/ttyUSB0" >&2
fi

# ---- 7. 监视窗口 ----
# 报告图的名字就是"时间 + 飞行报告"，每趟一份、不互相覆盖。
MON_OUT="/logs/$(date "+%Y-%m-%d %H%M") 飞行报告.png"
MON_LOG="/logs/${SCRIPT%.py}_real_monitor.log"
log "启动监视窗口（报告将存到 runtime_logs/$(basename "$MON_OUT")）"
# 监视器底图与航线：必须跟长机实际带队跑的那条逐点一致（含闭合段）。
# 2026-10-23 起由 scripts/route_for_monitor.py 从本目录的 venue.py 现算，
# **不再在这里抄第二份坐标**。改成现算的直接原因：核对时发现这里抄的那份
# 已经跟 venue.py 漂了 4 个点（A 写 2.5,2.5 实为 2.7,2.7；F 写 13.4,17 实为
# 10.0,17.5；C 写 17.5,22.5 实为 17.0,22.5；D 写 17.5,2.8 实为 17.2,2.7），
# 而航线对不上会让间距读成整条航线长（实测 67 米、占 38% 样本）。
# 算不出来就不传 --route：少一条间距曲线而已，不该让整个架次飞不了。
MON_ROUTE="$(python3 "$ROOT/scripts/route_for_monitor.py" "$HERE" "$SCRIPT" 2>&1)" || {
    echo "!! 航线算不出来（$MON_ROUTE），本架次监视器不画沿航线间距" >&2
    MON_ROUTE=""
}
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
# ---- --min-sep：把安全下限设到两机（飞机上）的 formation_follower_node ----
MON_MIN_SEP_OPT=()
if [ -n "$MIN_SEP" ]; then
    MON_MIN_SEP_OPT=(--min-sep "$MIN_SEP")
    echo "== 设置两机的直线距离安全下限 min_separation_m=$MIN_SEP =="
    _ms_out=$(docker run --rm --network host "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
        bash -lc "source /opt/ros/humble/setup.bash
            ok=1
            for ns in $LEADER $FOLLOWER; do
                node=/\$ns/formation_follower_node
                for i in \$(seq 1 10); do
                    ros2 node list 2>/dev/null | grep -qx \"\$node\" && break
                    sleep 1
                done
                got=
                for i in 1 2 3; do
                    err=\$(ros2 param set \$node min_separation_m $MIN_SEP 2>&1 | tail -1)
                    got=\$(ros2 param get \$node min_separation_m 2>/dev/null | tail -1)
                    case \"\$got\" in *$MIN_SEP*) break ;; esac
                    echo \"   \$ns 第 \$i 次没设上（set: \${err:-无输出} / 回读: \${got:-无响应}），重试\"
                    sleep 2
                done
                case \"\$got\" in
                    *$MIN_SEP*) echo \"   \$ns ✓ \$got\" ;;
                    *) echo \"   \$ns ✗ 最终没设上：\${got:-无响应}\"; ok=0 ;;
                esac
            done
            [ \$ok -eq 1 ]" 2>&1)
    _ms_rc=$?
    echo "$_ms_out" | grep -v '^== \[contestant-sdk\]'
    if [ $_ms_rc -ne 0 ]; then
        echo "!! min_separation_m 没能设到两机上 —— 这是两机不许靠太近的硬保护，" >&2
        echo "   值不确定就不飞。先确认两机飞行栈都起了、都在 ROS 域 20。" >&2
        exit 1
    fi
    # 等 DDS 收敛再起选手程序：紧接着起会报 takeoff:set_height 超时。
    sleep 5
fi

docker run -d --name "$C_MONITOR" --network host \
    -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -v "$ROOT/scripts:/scripts:ro" -v "$LOGDIR:/logs" \
    "${MON_LAYOUT_ARGS[@]}" \
    "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    python3 -u /scripts/monitor.py --leader "$LEADER" --follower "$FOLLOWER" \
        --spacing "$SPACING" "${MON_MIN_SEP_OPT[@]}" "${MON_LAYOUT_OPT[@]}" --route "$MON_ROUTE" \
        --out "$MON_OUT" >/dev/null 2>&1 \
    || { echo "!! 监视窗口没起来 —— 查 DISPLAY 和 xhost 授权" >&2
       echo "   不中止飞行，但这一轮没有编队间距记录和报告图" >&2; }

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

# 速度留空就不传：老镜像的 SDK 不认 --max-vel，argparse 会直接报错。
VEL_ARGS=()
if [ -n "$MAX_VEL" ]; then
    VEL_ARGS=(--max-vel "$MAX_VEL")
    vel_desc="限速=${MAX_VEL}m/s"
else
    vel_desc="限速=程序默认"
fi
log "启动选手程序：$SCRIPT（长机=$LEADER 僚机=$FOLLOWER 间距=${SPACING}米 $vel_desc）"
# CONTEST_CONTROL_HOST：SDK 的 set_camera_mode() 靠它找自己飞机的机载服务。
docker run -d --name "$C_LEADER" -e CONTEST_CONTROL_HOST="$LEADER_IP" "${COMMON[@]}" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$LEADER" --role leader \
    --teammate "$FOLLOWER" --spacing "$SPACING" "${VEL_ARGS[@]}" >/dev/null
docker run -d --name "$C_FOLLOWER" -e CONTEST_CONTROL_HOST="$FOLLOWER_IP" "${COMMON[@]}" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$FOLLOWER" --role follower \
    --teammate "$LEADER" --spacing "$SPACING" "${VEL_ARGS[@]}" >/dev/null

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
    # 先落盘再摘要：下面 cleanup 会 docker rm -f，不存下来异常原文就没了。
    docker logs "$c" > "$LOGDIR/${SCRIPT%.py}_real_$c.log" 2>&1 || true
    grep -E 'Traceback|Error|错误|失败' "$LOGDIR/${SCRIPT%.py}_real_$c.log" | tail -5 || true
done
docker logs "$C_MONITOR" > "$LOGDIR/${SCRIPT%.py}_real_monitor.log" 2>&1 || true
echo
echo "报告图：runtime_logs/$(basename "$MON_OUT")"
echo "本趟归档：runtime_logs/archive/${SCRIPT%.py}_$RUN_STAMP/（日志+CSV+报告图，不会被下一趟覆盖）"
echo "机载栈仍在运行（本脚本不拆机载容器）。要停机载栈走地面站网页或各机的 control_server。"

if [ "$KEEP" = "0" ]; then
    docker rm -f "$C_LEADER" "$C_FOLLOWER" "$C_MONITOR" >/dev/null 2>&1 || true
else
    echo "（--keep：选手容器已保留，翻完日志后 docker rm -f $C_LEADER $C_FOLLOWER $C_MONITOR）"
fi
exit $rc
