#!/usr/bin/env bash
# 一键真机：在地面站跑选手程序控制两架真飞机。
#
# 参数就是**要跑的程序文件名**（本目录里的 .py，.py 可省略，可 Tab 补全）：
#
#   ./run_real.sh formation_slim      # 编队飞行
#   ./run_real.sh groundfire_slim     # 地面火情
#   ./run_real.sh highrise_slim       # 高层火情
#   ./run_real.sh mission_slim        # 三轮连贯（火情需**人工布置**，真机没有出题裁判）
#                                     #   2026-10-14 真机实飞稳定完成
#
#   2026-10-14：_lite 版已由用户删除，只剩这四个 _slim 版。
#
#   ./run_real.sh formation_slim --spacing 4.0 --min-sep 3.0
#                                     # 改编队目标间距，同时把两机直线距离的安全
#                                     # 下限往下调（这两个必须一起改，否则目标值
#                                     # 压在保护线上、僚机参考点会被频繁冻住）。
#                                     # --min-sep 走 ros2 param set 打到**飞机上**
#                                     # 的 formation_follower_node，只对本次飞行
#                                     # 有效；在这个脚本里设环境变量没用。
#
#   ./run_real.sh                     # 不给就跑 formation_slim.py
#   ./run_real.sh highrise_slim --keep # 结束后保留选手容器，便于翻日志
#   ./run_real.sh formation_slim --skip-clock-check  # 跳过时钟同步闸门（见下）
#   ./run_real.sh mission_slim --max-vel 0.4         # 临时压低巡航限速
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

SCRIPT=formation_slim.py   # 2026-10-14：原 formation_lite.py 已删
# 编队间距（米）。2026-10-07 由 4.0 改为 5.0（用户要求），跟 SDK 的
# DroneSDK.spacing_m / run(spacing_m=) 默认值保持一致——这里用 --spacing 传给
# 选手程序，会**覆盖** SDK 的默认值，所以两边必须一起改。监视器画的"目标间距"
# 虚线也用这个值。临时换：./run_real.sh <程序> --spacing 4.0
SPACING=5.0
# 两机**空间直线距离**的安全下限（米），对应飞机上 formation_follower_node 的
# min_separation_m：低于它僚机参考点会被冻住（"直线距离保护：两机仅 X m < Y m，
# 参考点冻结等待"），现象是僚机不跟队。2026-10-07 真机两机相撞（最小直线间距
# 0.03 m）之后加的硬保护。
# 留空 = 不动飞机上的值（飞机本地 .env 的 FORMATION_MIN_SEPARATION_M）。
# 🔴 **真机这边只能用 ros2 param set，环境变量没用**：formation_follower_node
# 跑在**飞机上**（Jetson 里的 flight-stack 容器，由机载 control_server 起、读的
# 是飞机本地那份 compose/.env），而这个脚本跑在地面站，只 docker run 选手程序和
# 监视器三个容器——在这里 export FORMATION_MIN_SEPARATION_M 根本到不了那个节点，
# 而且是**静默**没用（跟"真机 compose 不透传 = 参数永远默认值"同一类坑）。
# 好在节点每轮循环都重读这个参数（不是启动时缓存一次），所以 param set 立刻
# 生效、不用重启飞行栈（避免 UWB 被热重启搞坏那条老坑）。
# ⚠️ 只在**本次飞行**有效：飞行栈一重启就回到飞机本地 .env 的值。调参定下来
# 之后要固化，得改飞机上那份 .env。
# ⚠️ 想把 --spacing 调到 4.0 的话，这个值必须一起往下调（比如 3.0），否则目标
# 值正好压在保护线上，参考点会被频繁冻住。
MIN_SEP=
KEEP=0
TIMEOUT_S=1500
SKIP_CLOCK=0
# 经规划器航段的限速（m/s），透传给选手程序的 --max-vel，覆盖程序里的默认常量
# （比如 mission_lite.py 的 MISSION_MAX_VEL_MPS）。管 goto/goto_world/fly_route/
# lead_formation。
# ⚠️ **不管直飞**：goto_direct / return_home(direct=True) 走的是 precision_servo
# 的另一条链路，速度是 SDK 的 DIRECT_CRUISE_MPS（1.0 m/s），改这里不影响它。
# 留空（MAX_VEL=）则不透传，各程序用自己的默认值。
MAX_VEL=0.6   # 2026-10-13 用户要求：0.8 -> 0.6。这是**主控**，会覆盖 .env 的 LINE_CRUISE_MPS
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
        --min-sep) MIN_SEP="$2"; shift 2 ;;
        --keep)    KEEP=1; shift ;;
        --skip-clock-check) SKIP_CLOCK=1; shift ;;
        --max-vel) MAX_VEL="$2"; shift 2 ;;
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
# 这一趟的时间戳：归档目录用它，保证历次飞行不互相覆盖。
RUN_STAMP="$(date +%Y%m%d-%H%M%S)"

# 把这一趟的日志和监视器产物归档到带时间戳的目录。
# **放在 cleanup 里**（trap EXIT）而不是只放在正常收尾流程里：中途 Ctrl-C 或
# 异常退出时，正常收尾那段根本不会执行，容器被 docker rm -f 一清日志就没了——
# 2026-10-07 连着几次查故障翻不到当趟日志就是这么丢的。
archive_run() {
    local dir="$LOGDIR/archive/${SCRIPT%.py}_$RUN_STAMP"
    mkdir -p "$dir" 2>/dev/null || return 0
    local c
    for c in "$C_LEADER" "$C_FOLLOWER" "$C_MONITOR"; do
        docker logs "$c" > "$dir/$c.log" 2>&1 || true
        # 容器不存在时 docker logs 会把报错写进文件，留个空文件没意义，删掉
        [ -s "$dir/$c.log" ] || rm -f "$dir/$c.log"
        grep -q "No such container" "$dir/$c.log" 2>/dev/null && rm -f "$dir/$c.log"
    done
    # 监视器的 CSV/PNG：它自己写在 LOGDIR 下、每趟同名覆盖，这里拷一份进归档
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

# 同一时刻只允许一个 run_real.sh：两个并行会互相拆对方的选手容器。
LOCK=/tmp/docker_sim_run_real.lock
exec 9>"$LOCK"
flock -n 9 || die "已经有一个 run_real.sh 在跑（锁 $LOCK）。先等它结束或 ./stop.sh"

# ---- 1. 找程序 ----
SRCDIR="$HERE"
# 列举时排除 venue.py（那是坐标表，不是可跑的程序）。原来只列 *_lite.py，
# 结果 basic_test.py 这类测试程序不会出现在提示里。
[ -f "$HERE/$SCRIPT" ] || die "找不到程序 $SCRIPT（本目录下可选：$(cd "$HERE" && ls *.py 2>/dev/null | grep -v '^venue\|^rounds\.py$' | tr '\n' ' '))"
log "程序：$SRCDIR/$SCRIPT"

purge_stale

# ---- 1.8 程序代码一致性 ----
# 真机与仿真的选手程序**必须逐字节相同**，差异只允许出现在 venue.py（场地参数）。
# 这样改场地坐标不会把程序改出分叉，修 bug 也只需要改一边再同步。
# 漂移了就拦下——不然两套代码会悄悄长歪，到现场才发现就晚了。
SIMDIR="$ROOT/contestant_sim"
if [ -d "$SIMDIR" ]; then
    drift=""
    # 比**所有** .py，不只 *_lite.py：basic_test.py 这类场地无关的测试
    # 程序两边也该逐字节相同。venue.py 本来就该不同，跳过。
    # 顶层 + 单机测试/ 都比。venue.py 本来就该不同，跳过。
    for f in "$HERE"/*.py "$HERE"/单机测试/*.py; do
        [ -e "$f" ] || continue
        b="${f#$HERE/}"                 # 保留相对路径，便于区分同名文件
        [ "$(basename "$f")" = venue.py ] && continue
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
# ---- 共享栈必起（2026-10-03 用户要求）----
# 声光栈和网页栈都不属于本脚本自己的容器，原来 check_env.sh 对它们只给 △
# 提示不拦。真机侧两个都必起。放在 check_env 之前，让它复核修好之后的状态。
source "$ROOT/scripts/ensure_stacks.sh"
ensure_sound_light real
ensure_gcs_web

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

# ---- 3.5 时钟同步闸门（2026-10-10 加）----
# 为什么必须拦：ego_planner 交换 swarm 轨迹时，对方的时间戳比本机晚 0.25 秒以上
# 就把整条轨迹丢弃。10-09 两机时钟源不统一（地面站自己没有上游、NX02 把它判成
# false ticker 转而去对外网），两机差 0.45~0.52 秒，编队协同全程失效——而机载
# 日志里只表现为"轨迹被丢弃"，现场完全看不出是时钟的事，白飞了一下午。
# 断电重启后 chrony 会重新选源，所以**每趟都要查**，不是配完那次查过就完了。
# 代价：两次 ssh，实测 0.3 秒。
GCS_IP=192.168.2.103
CLOCK_MAX_OFFSET_S=0.05   # 单机 RMS 偏移上限。两机各 0.05 → 相对 0.1，仍远低于 0.25
if [ "$SKIP_CLOCK" = 1 ]; then
    log "△ 跳过时钟同步检查（--skip-clock-check）——编队协同可能失效"
else
    # chrony 的 Reference ID 是当前选中源的 IP 十六进制。两机必须都选地面站，
    # 只看"偏移小"不够：各自对不同的源也可能都显示偏移小，但彼此差一大截。
    want_hex=$(LC_ALL=C python3 -c \
        'import sys;print("".join("%02X"%int(x) for x in sys.argv[1].split(".")))' "$GCS_IP")
    for ip in "$LEADER_IP" "$FOLLOWER_IP"; do
        # 机载 locale 是印尼语，awk 不把 . 当小数点，两边都得钉 LC_ALL=C
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
# shellcheck source=/dev/null
source "$ROOT/scripts/contestant_network.sh"
contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" || die "真机网络参数生成失败"
log "网络：$CONTESTANT_NET_DESC"

# ---- 机载 MAVLink 流收窄（降 CPU，2026-10-03）----
# 飞控 SD 卡把 ODOMETRY / LOCAL_POSITION_NED / ATTITUDE_QUATERNION /
# HIGHRES_IMU 四路顶到了 100Hz，实测白占机载整机 8-9 个百分点的 CPU。改那四
# 行要拆机取 SD 卡，所以在这里用 MAV_CMD_SET_MESSAGE_INTERVAL(511) 运行时压
# 回来。每一路关/降的依据和完整实测数据见 scripts/tune_mavlink_streams.sh
# 文件头。失败只警告不中止（不收窄也能飞，只是 CPU 高一截）。
# 必须放在 contestant_net_args 之后——它要靠 CONTESTANT_NET_ARGS 进真机域。
source "$ROOT/scripts/tune_mavlink_streams.sh"
log "收窄机载 MAVLink 高频流（降 CPU）"
tune_mavlink_streams "$LEADER" "$FOLLOWER"

# ---- 5. 相机：不在这里配 ----
# 用哪一路相机**由程序自己声明**（开头调 sdk.set_camera_mode()）。SDK 从
# 2026-10-03 起在 set_camera_mode() 里做**两路互斥**：开一路之前先停掉另一路，
# 保证"每架飞机同时最多只有一路检测在跑"；并且等到那一路真出数据才返回，失败
# 抛异常。在那之前互斥只做在 mission_lite 的 _use_camera() 里，5 个单任务程序
# 只"加开自己那路"，于是地面站起 vision 栈时默认拉起的那一路会一直挂着——实测
# NX02 默认开下视、跑 highrise_lite（要前视）之后两路都在跑，整机 88.7%。
# 这里曾经有一套"按程序文件名猜需要哪路、运行器替它配好"的机制，拆掉了：
#   · 按文件名匹配（*groundfire*|*抓取*…）本身就脆，改个名就失效，新程序要
#     回来改表；
#   · 程序自己最清楚用哪一路，声明在代码里比写在运行器的表里更不容易错；
#   · set_camera_mode(wait=True) 本身就验证了"这一路真在出数据"，不需要运行器
#     再单独验一遍。
# 需要的只是"能动态起停"这个能力，那在 SDK 里。

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
# 俯视底图：monitor.py 不给 --layout 就退回写死的 fire_drill_room 老坐标
# （房间原点在**中心**、立柱 (4.5,7)），跟样题场景完全不是一套，轨迹会叠在
# 错误的底图上。样题布局的权威来源是 src/contest_mission/config 下那份 yaml
# （room/pillars/pads/obstacle_cylinder/terrain_module 都在里面）。
# 2026-10-05 补：之前一直没传，每趟监视器都在打那条"没给 --layout"的警告。
# 沿航线间距的"尺子"：两机都投影到这条折线上，间距 = 弧长差（同段内就是
# 直线距离，跨段就是经过航点那几段之和）。2026-10-15 补上：以前真机从来不传
# --route，监视器退回用直线距离，报告里"沿航线间距"和"空间直线距离"两条
# 线一直是重合的（指纹：CSV 里 gap_route == gap_line）。
# 坐标用 contestant_real/venue.py 的**实测校准值**，改了那边记得同步这里。
# ⚠️ 必须跟长机实际带队跑的那条逐点一致；尤其是**最后回 A 那一段不能漏**
#   （lead_formation 会把 final_xy 接到尾巴上）——monitor 的 RoutePath.is_closed()
#   靠首尾距离 <0.5 m 判闭合，不闭合就不走"环上取短边"保护，两机停在
#   起降点上时间距会读成整条航线长（仿真那边实测 67 m，占全程 38% 的样本）。
#   A B C G E F G D A（formation_slim / mission_slim 第 1 轮跑的全环，总长 88.9 m）
MON_ROUTE_LOOP="2.5,2.5 2.5,22.5 17.5,22.5 17.5,12.5 10,12.5 13.4,17 17.5,12.5 17.5,2.8 2.5,2.5"
#   G D A（groundfire/highrise 和 mission 第 2/3 轮的 formation_home：只带队跑
#   [G, D] 再解散回 A，不是闭合环，总长 24.7 m）
MON_ROUTE_TAIL="17.5,12.5 17.5,2.8 2.5,2.5"
case "$SCRIPT" in
    formation*|mission*) MON_ROUTE="$MON_ROUTE_LOOP" ;;
    *)                   MON_ROUTE="$MON_ROUTE_TAIL" ;;
esac
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
# 放在起监视器**之前**：监视器画的那条红线要跟飞机上实际生效的值一致。
#
# ⚠️ 为什么是"一个容器里把两机都设了" + "先等节点出现" + "失败重试"：
# 2026-10-19 第一版是每台各起一个 `docker run --rm` 容器，实测 NX01 成功、
# NX02 **set 超时失败而随后的 get 成功**，于是回读到旧值 4.0、脚本判定失败。
# 根因是新容器每次都要重做 DDS 服务发现，第一次 param set 赶在发现完成之前就
# 超时了（几百毫秒后的 get 就正常）。所以：一次容器调用里顺序处理两机（发现
# 只做一次、连接已经热了），调服务前先用 node list 等节点真的出现，再给 set
# 三次重试，并且**不吞 set 的错误输出**——第一版 `>/dev/null 2>&1` 把真实原因
# 全吃掉了，只能看到一句没头没尾的"没设上"。
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
    # ⚠️ 退出后留一点时间让 DDS 收敛再往下走。2026-10-19 实测：这一步的临时
    # 容器 11:15:34 刚退出，选手程序紧接着启动、5 秒内没发现 pt4ctrl 的
    # set_parameters 服务，takeoff 第一步就报 `takeoff:set_height` 超时
    # （历史归档里从没出现过这个错，加了这一步当天就出现）。participant 离开
    # 会让域里所有节点更新发现信息，而这套真机环境的发现本来就慢——同一时段
    # `ros2 node list` 第一次只看得到 NX01，等 5 秒才看到 NX02 那 27 个节点。
    # 代价只是多等几秒，换掉一次"飞不起来还看不出为什么"。
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

# 速度留空就不传：老镜像的 SDK 不认 --max-vel，argparse 会直接报错起不来。
VEL_ARGS=()
if [ -n "$MAX_VEL" ]; then
    VEL_ARGS=(--max-vel "$MAX_VEL")
    vel_desc="限速=${MAX_VEL}m/s"
else
    vel_desc="限速=程序默认"
fi
log "启动选手程序：$SCRIPT（长机=$LEADER 僚机=$FOLLOWER 间距=${SPACING}米 $vel_desc）"
# CONTEST_CONTROL_HOST：SDK 的 set_camera_mode() 靠它找自己飞机的
# control_server(8890) 切相机模式。**不注入的话 SDK 会判成仿真、空操作**
# （仿真没有 control_server，检测节点随飞行栈起、本来不用切），于是真机上
# 的切换会静默不生效。各容器注入**自己那架**的 IP。
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
    # 2026-10-04 补（run_test.sh 那边同日修了同类问题，那边更严重：它 grep 的
    # 是一个从没被生成的文件，摘要恒为空）。
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
