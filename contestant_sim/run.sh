#!/usr/bin/env bash
# 一键仿真：主机开机后跑这一个脚本就够了。
#
# 参数就是**要跑的程序文件名**（本目录里的 .py，.py 可省略，可 Tab 补全）：
#
#   ./run.sh formation_slim    # 编队飞行
#   ./run.sh groundfire_slim   # 地面火情：侦查 -> 取物资 -> 投弹 -> 编队返航
#   ./run.sh highrise_slim     # 高层火情：巡检拍摄 -> 协同灭火 -> 编队返回
#   ./run.sh mission_slim      # 三轮连贯：编队 + 两种火情（火情随机、两轮不重复）
#
#   2026-10-14：原版(formation.py 等)和 _lite 版已由用户删除，只剩这四个
#   _slim 版（航线常量收进 rounds.py，四个任务共用一份）。
#
#   ./run.sh                   # 不给就跑 formation_slim.py
#   ./run.sh mission_slim --max-vel 0.6     # 临时压低巡航限速
#   ./run.sh highrise_slim --keep           # 结束后保留选手容器，便于翻日志
#   ./run.sh formation_slim --spacing 4.0 --min-sep 3.0
#                                           # 改编队目标间距，同时把两机直线距离的
#                                           # 安全下限往下调（这两个必须一起改，
#                                           # 否则目标值压在保护线上、僚机跟不了队）
#
# 用文件名当参数，是因为它本来就是唯一且无歧义的标识：不用再维护一张
# "场景名 -> 文件名"的映射表，加新程序也不用动这个脚本。
#
# 它做这些事（按顺序）：
#   1. 检查 docker / 镜像 / X11
#   2. 起仿真容器（Gazebo + RViz 由 .env 的 USE_GAZEBO_GUI/USE_RVIZ 自动拉起）
#   3. 等两机就绪（ROS 节点 + PX4 自检都过）
#   4. 起监视窗口（实时俯视轨迹 / 高度 / 间距 + 飞行统计）
#   5. 跑选手程序（长机僚机各一个容器）
#   6. 收尾：让监视存图、修正日志属主、打印结果
#
# 程序按名字在本目录找：你自己写的程序，和四个 *_lite.py。
# 本目录被整个挂成容器里的 /workspace，所以同目录的依赖（venue.py、彼此之间
# 的 import）就地解析。
# monitor.py / referee.py 直接从 scripts/ 取。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # 选手目录 = 本脚本所在目录
ROOT="$(cd "$HERE/.." && pwd)"                       # docker_sim 仓库根（仿真设施）

SCRIPT=formation_slim.py   # 2026-10-14：原 formation.py 已删
# 编队间距（米）。2026-10-12 由 4.0 同步成 5.0，跟真机 run_real.sh 的 SPACING
# 和 SDK 的 DroneSDK.spacing_m 默认值对齐（真机 2026-10-07 就改了 5.0，仿真一直
# 没跟）。这里用 --spacing 传给选手程序，会**覆盖** SDK 默认值；监视器画的
# "目标间距"虚线也用这个值。临时换：./run.sh <程序> --spacing 4.0
SPACING=5.0
# 经规划器航段的巡航限速（m/s），透传给选手程序的 --max-vel，覆盖程序里的兜底常量
# （mission_lite 的 MISSION_MAX_VEL_MPS / highrise_lite 的 HIGHRISE_MAX_VEL_MPS，
# 都是 0.8）。管 goto/goto_world/fly_route/lead_formation。
# ⚠️ **不管直飞**：goto_direct / return_home(direct=True) 走 precision_servo 的
# 另一条链路，速度是 SDK 的 DIRECT_CRUISE_MPS(1.0)，改这里不影响它。
# 2026-10-12 新增并定为 1.0（用户要求把仿真最大飞行速度提到 1 m/s）。在此之前
# 仿真运行器**根本没有这个开关**，于是 sdk.max_vel_mps=None、三个程序一律退回
# 各自 0.8 的兜底常量——而真机 run_real.sh 早就是 MAX_VEL=1.0，两边一直差着
# 20%，却看不出来（限速写在选手程序里，两侧逐字节相同，差异藏在运行器）。
# 另外僚机的参考点推进速率（cruise_speed_mps）现在也跟着这个值走，见
# follow_formation() 的 cruise_mps 参数。临时改：./run.sh <程序> --max-vel 0.6
MAX_VEL=1.0
# 两机**空间直线距离**的安全下限（米），对应 formation_follower_node 的
# min_separation_m：低于它僚机的参考点会被冻住（日志里那句"直线距离保护：
# 两机仅 X m < Y m，参考点冻结等待"），现象是僚机不跟队。
# 2026-10-07 真机两机相撞（最小直线间距 0.03 m）之后加的硬保护。
# 留空 = 不动飞控侧的值（用 .env 的 FORMATION_MIN_SEPARATION_M，现 4.0），
# 监视器那条红色下限线按 monitor.py 自己的默认值 4.0 画。
# 传了就**同时**做两件事：① ros2 param set 到两机节点（节点每轮循环都重读
# 这个参数，所以立刻生效、不用重启飞行栈）；② 传给监视器画线——两者用同一个
# 值，避免"图上画 4.0、飞控其实按 3.0 在守"这种对不上的情况。
# ⚠️ 想把目标间距 --spacing 调到 4.0 的话，这个值必须一起往下调（比如 3.0），
# 否则目标值正好压在保护线上，参考点会被频繁冻住、僚机根本跟不了队。
MIN_SEP=
RESTART=1
KEEP=0
TIMEOUT_S=1500
while [ $# -gt 0 ]; do
    case "$1" in
        --spacing)       SPACING="$2"; shift 2 ;;
        --min-sep)       MIN_SEP="$2"; shift 2 ;;
        --max-vel)       MAX_VEL="$2"; shift 2 ;;
        --no-restart)    RESTART=0; shift ;;
        --keep)          KEEP=1; shift ;;
        --timeout)       TIMEOUT_S="$2"; shift 2 ;;
        -h|--help)       sed -n '2,26p' "$0"; exit 0 ;;
        -*) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
        *)               SCRIPT="${1%.py}.py"; shift ;;   # 程序文件名，.py 可省
    esac
done

# 航线（世界坐标）。两个用处，别混：
#   ① 给监视器 --route，它把两机投影到这条折线上、取弧长差当"沿航线间距"；
#   ② 只有老版 formation.py 把它当航点吃进去（见下面 EXTRA），_slim 那四个
#      的航点写在自己代码里，传了也不吃。
# ⚠️ 必须跟"长机实际带队跑的那条"逐点一致，否则间距曲线整条失真。
#   2026-10-15 修：原值 "3,3 3,22 17,22 17,16 10,16 14,14 17,16 17,3" 三处过时——
#   G/E 的 y 2026-10-11 就从 16 挪到 12.5、F 是 (13.5,16.5) 不是 (14,14)，
#   **尾巴还漏了回 A 那一段**（lead_formation 会把 final_xy=ROUTE_A 接上，
#   实际是 9 点闭合回路）。漏掉收尾那段的后果最重：RoutePath.is_closed()
#   靠首尾距离 <0.5 m 判闭合，不闭合就不走"环上取短边"那条保护，于是
#   两机还停在各自起降点上时间距读成 67.08 m（= 整条航线长）、解散后
#   回家那段又读 67 m，实测占全程 38% 的样本（2026-10-15 用户发现）。
#   反向伪值也有：两机都投影到同一个末端时读 0.40 m（实际相距 5.82 m）。
#   坐标权威来源是 contestant_sim/venue.py，改了那边记得同步这里。
#   A B C G E F G D A（formation_slim / mission_slim 第 1 轮跑的全环）
ROUTE_LOOP="3,3 3,22 17,22 17,12.5 10,12.5 13.5,16.5 17,12.5 17,3 3,3"
#   G D A（groundfire/highrise 和 mission 第 2/3 轮的 formation_home，
#   它只带队跑 [G, D] 再解散回 A，不是闭合环）
ROUTE_TAIL="17,12.5 17,3 3,3"

# 按程序选航线：哪条都行不通，四个程序的编队段本来就不是同一条。
case "$SCRIPT" in
    formation*|mission*) ROUTE="$ROUTE_LOOP" ;;
    *)                   ROUTE="$ROUTE_TAIL" ;;
esac

LEADER=NX01
FOLLOWER=NX02
IMAGE=contestant-sdk:latest
FSNX01=docker_sim-flight-stack-nx01-1
SIMWORLD=docker_sim-sim-world-1
LOGDIR="$ROOT/runtime_logs"
LAYOUT=/opt/contest_mission_ws/src/contest_mission/config/sample_room_layout.yaml
C_LEADER=sim_leader
C_FOLLOWER=sim_follower

log() { echo "[$(date +%H:%M:%S)] $*"; }

# ---- 互斥锁：同一时刻只允许一个 run.sh 操作这套仿真 ----
# 两个 run.sh 并行会互相拆对方的容器（compose down 是全局的），现象是飞到
# 一半仿真突然没了、或者监视/裁判连到了另一轮的飞机上。2026-10-01 收尾时
# 发现上一轮摔机后 run.sh 还在后台等一个永远等不到的容器，就是靠这个锁
# 能第一时间看出来。
LOCK=/tmp/docker_sim_run.lock
exec 9>"$LOCK"
if ! flock -n 9; then
    holder="$(cat "$LOCK" 2>/dev/null)"
    echo "!! 已经有一个 run.sh 在跑这套仿真（PID ${holder:-未知}）。" >&2
    echo "   等它结束，或者 kill ${holder:-<PID>} 之后再来。" >&2
    exit 1
fi
echo $$ >&9
cleanup() { [ "$KEEP" = "1" ] && return 0; docker rm -f "$C_LEADER" "$C_FOLLOWER" >/dev/null 2>&1 || true; }
trap cleanup EXIT

[ -f "$ROOT/scripts/monitor.py" ] || { echo "!! 缺 scripts/monitor.py !!" >&2; exit 1; }

echo "=============================================="
echo " 一键仿真：$SCRIPT"
echo " 目录 $HERE"
echo "=============================================="

# ---- 1. 前置检查 ----
command -v docker >/dev/null || { echo "!! 没装 docker !!" >&2; exit 1; }
docker info >/dev/null 2>&1 || {
    echo "!! docker 守护进程没在跑。先 sudo systemctl start docker !!" >&2; exit 1; }
[ -f "$ROOT/docker-compose.yml" ] || {
    echo "!! $ROOT 下没有 docker-compose.yml——本目录要放在 docker_sim 仓库里 !!" >&2; exit 1; }
for img in "$IMAGE" flight-stack:latest sim-world:latest; do
    docker image inspect "$img" >/dev/null 2>&1 || {
        echo "!! 镜像 $img 不存在。先在 $ROOT 下 build（见 docker-compose.yml 顶部说明） !!" >&2
        exit 1; }
done

# X11：容器里的 Gazebo/RViz/监视窗口都要往宿主机的 X server 上画。
# DISPLAY 没设时给个默认值——从 tty 或 ssh 进来跑的话它可能是空的。
export DISPLAY="${DISPLAY:-:1}"
if command -v xhost >/dev/null; then
    xhost +local: >/dev/null 2>&1 || echo "（xhost 授权没成功，窗口可能弹不出来）" >&2
fi
log "DISPLAY=$DISPLAY"

# ---- 1.5 真机残留检查（2026-10-03 新增）----
# 仿真和真机主要靠 ROS_DOMAIN_ID 隔离（仿真 21 / 真机 20），话题不会串；容器名
# 和锁也是分开的。但有两类东西隔离不了，必须在起仿真之前拦住：
#   · 声光常驻程序只有一个容器、只能在一个域——它要是还停在真机域 20，
#     仿真这边的播报会全部静默失效（不报错，只是没声音）。
#   · 真机的选手容器要是还在跑，它们在对两架真飞机发指令，而你正准备用
#     同一台机器起 Gazebo——CPU 被挤之外，真机那边也不该无人照看。
# ---- 共享栈必起（2026-10-03 用户要求）----
# 声光栈不属于本脚本自己的容器，原来 check_env.sh 只给 △ 提示不拦，于是可以
# 一声不响飞完整轮、事后才发现任务通报全缺。改成必起。放在 check_env 之前，
# 让 check_env 去复核修好之后的状态。
# 网页栈仿真侧**不管**（用户明确：它只用于观察）。
source "$ROOT/scripts/ensure_stacks.sh"
ensure_sound_light sim

"$ROOT/scripts/check_env.sh" sim || { echo "!! 环境检查未通过（见上），处理后再跑" >&2; exit 1; }

# ---- 2. 定位程序：按名字去源目录找，不保留副本 ----
# 2026-10-01：以前 run.sh 住在单独的"一键仿真"目录里，程序要先同步一份副本
# 过去才能跑——两份真相，同步只是在给它打补丁，而且每次改代码都得来回 cd。
# 现在脚本跟程序待在同一个目录，按名字就地解析。
# 只在本目录找：你自己写的程序，和四个 *_lite.py。
# 本目录会被整个挂成容器里的 /workspace，所以同目录的依赖（venue.py、彼此
# 之间的 import）就地解析，不用另挂。
SRCDIR=""
[ -f "$HERE/$SCRIPT" ] && SRCDIR="$HERE"
if [ -z "$SRCDIR" ]; then
    echo "!! 找不到 $SCRIPT。可跑的程序：" >&2
    # `|| true`：目录里一个 .py 都没有时 ls 失败，会被 `set -eo pipefail`
    # 当成致命错误，列举还没开始就退出了。
    ( cd "$HERE" && ls -1 *.py 2>/dev/null \
      | grep -v '^monitor\.py$\|^referee\.py$\|^venue\.py$\|^rounds\.py$' \
      | sed "s|^|     |" ) >&2 || true
    exit 1
fi
log "程序：${SRCDIR#$ROOT/}/$SCRIPT"

# ---- 2.5 静态自检：语法之外，还查"调用了不存在的函数"这类运行时才炸的错 ----
if [ -f "$ROOT/scripts/check_python_static.py" ]; then
    if ! python3 "$ROOT/scripts/check_python_static.py" "$SRCDIR"/*.py; then
        echo "!! ${SRCDIR#$ROOT/} 里的程序静态自检不通过，先修好再飞 !!" >&2
        exit 1
    fi
fi
# 装了 pyright 就顺便做一次类型检查（参数名写错、传错类型这类）。
# 没装就跳过——它是可选的，不该成为飞行的硬依赖。
if command -v pyright >/dev/null 2>&1 && [ -f "$HERE/pyrightconfig.json" ]; then
    if ! ( cd "$HERE" && pyright --outputjson >/dev/null 2>&1 ); then
        echo "（pyright 发现类型问题，不拦飞行；细看跑一次 cd $HERE && pyright）" >&2
    fi
fi
# ---- 3. 起仿真 ----
docker rm -f "$C_LEADER" "$C_FOLLOWER" >/dev/null 2>&1 || true
if [ "$RESTART" = "1" ]; then
    log "重启仿真容器（WORLD_ENV=sample_room）…"
    ( cd "$ROOT" && WORLD_ENV=sample_room docker compose down --timeout 20 >/dev/null 2>&1 || true )
    ( cd "$ROOT" && WORLD_ENV=sample_room docker compose up -d >/dev/null )
fi

# ---- 4. 等两机就绪 ----
# 不能写成 `docker logs | grep -q`：pipefail 下 grep 命中就退出，docker logs
# 还在往管道写、被 SIGPIPE 打死（141），匹配成功反而判成失败，日志越大越必然。
log_has() { local out; out="$(docker logs "$1" 2>&1 || true)"; case "$out" in *"$2"*) return 0;; *) return 1;; esac; }
px4_ready() {
    for f in /tmp/px4_NX01.log /tmp/px4_NX02.log; do
        docker exec "$SIMWORLD" grep -q "Ready for takeoff" "$f" 2>/dev/null || return 1
    done
}
log "等两机就绪（ROS 节点 + PX4 自检）…"
DL=$(( $(date +%s) + 300 ))
while :; do
    ok=1
    for c in nx01 nx02; do log_has "docker_sim-flight-stack-$c-1" 'formation_follower_node就绪' || ok=0; done
    px4_ready || ok=0
    [ "$ok" = "1" ] && break
    if [ "$(date +%s)" -ge "$DL" ]; then
        echo "!! 等仿真就绪超时。看 docker compose logs 排查 !!" >&2; exit 1
    fi
    sleep 5
done
log "两机就绪"

# Gazebo / RViz 由 sim-world 容器按 .env 的 USE_GAZEBO_GUI / USE_RVIZ 自己拉起，
# 这里只确认一下，没起来就提示（不致命，不影响飞行）。
sleep 2
gz_n="$(docker exec "$SIMWORLD" bash -lc 'pgrep -c gzclient || true' 2>/dev/null | tr -d '\r')"
rv_n="$(docker exec "$SIMWORLD" bash -lc 'pgrep -c rviz2 || true' 2>/dev/null | tr -d '\r')"
log "Gazebo 界面 ${gz_n:-0} 个、RViz ${rv_n:-0} 个"
[ "${gz_n:-0}" = "0" ] && echo "（Gazebo 界面没起：检查 .env 的 USE_GAZEBO_GUI=true 和 xhost 授权）" >&2
[ "${rv_n:-0}" = "0" ] && echo "（RViz 没起：检查 .env 的 USE_RVIZ=true）" >&2

# ---- 5. 监视窗口 ----
# docker cp 送进容器再跑，保证用的是**本目录这一份**监视程序
# （compose 把 ROOT/scripts 挂在 /opt/host_scripts，那是另一份）。
# 综合任务是多轮的：轮次之间任务机降落、侦察机在 A 点悬停，正好满足监视默认的
# "任务结束"判据，会在半道退出（用户 2026-09-30 发现"监控也自己掉了"）。
# 侦察机只在整个任务结束时才降落，拿它当判据最准。
MON_END=""
# mission*.py 是多轮流程，收尾判据要用"两机都降落"（见 monitor.py 里的说明）
case "$SCRIPT" in mission*.py) MON_END="--end-on-all-land" ;; esac
# ---- --min-sep：把安全下限设到两机的 formation_follower_node 上 ----
# 放在起监视器**之前**：监视器画的那条红线要跟飞控侧实际生效的值一致。
#
# ⚠️ 两机在**一次** docker exec 里一起设（NX01 的容器跟 NX02 同一个 ROS 域，
# 看得见对方的节点），而且调服务前先等节点出现、失败重试三次、不吞 set 的
# 错误输出。理由见 run_real.sh 里同一段的长注释：2026-10-19 真机上第一版
# 每台各起一个容器，NX02 的 param set 赶在 DDS 服务发现完成之前超时，而随后
# 的 get 成功、回读到旧值，脚本只能报一句没头没尾的"没设上"。
MON_MIN_SEP_ARGS=""
if [ -n "$MIN_SEP" ]; then
    MON_MIN_SEP_ARGS="--min-sep $MIN_SEP"
    log "设置两机的直线距离安全下限 min_separation_m=$MIN_SEP"
    _ms_out=$(docker exec "$FSNX01" bash -lc "
        source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0 \
               RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
               CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml
        ok=1
        for ns in NX01 NX02; do
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
    echo "$_ms_out"
    if [ $_ms_rc -ne 0 ]; then
        echo "!! min_separation_m 没能设到两机上 —— 这是两机不许靠太近的硬保护，" >&2
        echo "   值不确定就不飞。先确认两机 flight-stack 容器都在跑。" >&2
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

MON_OUT="/logs/${SCRIPT%.py}_formation.png"
MON_LOG="/logs/${SCRIPT%.py}_monitor.log"
log "启动监视窗口（报告将存到 runtime_logs/$(basename "$MON_OUT")）"
docker cp "$ROOT/scripts/monitor.py" "$FSNX01:/tmp/monitor.py" >/dev/null 2>&1 || true
docker exec -d -e DISPLAY="$DISPLAY" "$FSNX01" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0 \
           RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
           CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml
    python3 -u /tmp/monitor.py --layout '$LAYOUT' --route '$ROUTE' \
        --out '$MON_OUT' --spacing $SPACING $MON_MIN_SEP_ARGS $MON_END > '$MON_LOG' 2>&1
" >/dev/null 2>&1 || { echo "!! 监视窗口没起来 —— 查 DISPLAY 和 xhost 授权" >&2
       echo "   不中止飞行，但这一轮没有编队间距记录和报告图" >&2; }

# ---- 5.2 EKF 健康标志 / 飞行模式录制 ----
# 2026-10-17：录 mavros/state（飞行模式）和 mavros/estimator_status（EKF 健康位）。
# 为什么要录：仿真里"飞着飞着被放到地上"那类故障，直接原因是 PX4 的
# mc_pos_control 打 `invalid setpoints` 然后 `Failsafe: blind land`，而它的上游链是
#   位置估计被判不可用 → 从 OFFBOARD 降级（COM_OBL_RC_ACT=0 本指向 POSCTL，
#   但 POSCTL 要水平位置，于是继续降到 ALTCTL/nav_state=1）
#   → ALTCTL 的 flight task 需要遥控器输入、仿真里没有遥控器（日志里那句
#     "Matching flight task was not able to run"）→ setpoint 全 NaN → 判无效。
# 事后拿这两份录制对下面的正常基线，**哪一位从 true 翻成 false 就直接指出断的是
# 哪条估计链**，不用再像 2026-10-17 那样反推一整夜：
#   pos_horiz_rel / pred_pos_horiz_rel / pos_vert_abs / velocity_horiz / velocity_vert = true
#   pos_horiz_abs / pred_pos_horiz_abs = false  ← 正常！没有 GPS，位置靠 vision
#   accel_error / gps_glitch / const_pos_mode  = false
#   mode 全程只应有 OFFBOARD 和 AUTO.LOITER（起降前后），**出现 ALTCTL 就是中招**
# 两个话题各 1~2 Hz，一轮几百 KB，开销可忽略。EKF_RECORD=0 可关掉。
# ⚠️ 必须 `docker exec -d`：用自写的 rclpy 探针试过三次，都被
#    ExternalShutdownException 打死；`ros2 topic echo` 即使中途被杀，已写入的行也在。
if [ "${EKF_RECORD:-1}" != "0" ]; then
    for ns in "$LEADER" "$FOLLOWER"; do
        c="docker_sim-flight-stack-$(echo "$ns" | tr 'A-Z' 'a-z')-1"
        for t in state estimator_status; do
            docker exec -d "$c" bash -lc "
                source /opt/ros/humble/setup.bash
                export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0 \
                       RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
                       CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml
                exec ros2 topic echo --no-arr /${ns}/mavros/${t} \
                     > /logs/${SCRIPT%.py}_rec_${t}_${ns}.txt 2>&1
            " >/dev/null 2>&1 || true
        done
    done
    log "EKF/模式录制已起（runtime_logs/${SCRIPT%.py}_rec_*.txt）"
fi

# ---- 5.5 综合任务：起"出题裁判" ----
# 它负责把两处火情标识先从 world 里删掉，等侦察机过 G 点再把本轮抽中的那个
# 生成回来（两轮不重复）。只有综合任务需要——三个单任务的火情是固定摆好的。
case "$SCRIPT" in mission*.py) NEED_REFEREE=1 ;; *) NEED_REFEREE=0 ;; esac
if [ "$NEED_REFEREE" = "1" ]; then
    log "启动出题裁判（火情随机出现，两轮不重复）"
    docker cp "$ROOT/scripts/referee.py" "$SIMWORLD:/tmp/referee.py" >/dev/null 2>&1 || true
    docker exec -d "$SIMWORLD" bash -lc "
        source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0 \
               RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
               CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml
        python3 -u /tmp/referee.py --leader $LEADER > /logs/mission_referee.log 2>&1
    " >/dev/null 2>&1 || echo "（裁判没起来，火情不会随机出现）" >&2
    sleep 3
    docker exec "$SIMWORLD" head -2 /logs/mission_referee.log 2>/dev/null || true
fi

# ---- 6. 跑选手程序 ----
# 本目录挂 /workspace。程序要 import venue，本目录整个挂成 /workspace，
# 同目录的依赖就地解析；
# /etc/localtime 挂进去，照片时间戳和统计里的时刻才是本地时间（否则是 UTC）。
# PYTHONPATH 必须**在镜像原值后面追加**，不能直接覆盖：镜像 ENV 里带着
# /opt/quadrotor_msgs_install/...，`-e PYTHONPATH=/workspace:/deps` 会把它整个
# 顶掉，飞起来就是 ModuleNotFoundError: No module named 'quadrotor_msgs'。
BASE_PYPATH="$(docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$IMAGE" \
               | sed -n 's/^PYTHONPATH=//p' | head -1)"
PYPATH="/workspace${BASE_PYPATH:+:$BASE_PYPATH}"
PYPATH="${PYPATH%:}"      # 镜像原值自带尾部冒号，空条目会把 CWD 也塞进 sys.path
COMMON=(--network host
        -v /etc/localtime:/etc/localtime:ro
        -v "$SRCDIR:/workspace:ro"
        -v "$LOGDIR:/logs"
        -e PYTHONPATH="$PYPATH")
EXTRA=()
# 只有老版 formation.py 吃 --route；其余程序的航点写在自己代码里
[ "$SCRIPT" = "formation.py" ] && EXTRA=(--route "$ROUTE")

VEL_ARGS=()
if [ -n "$MAX_VEL" ]; then
    VEL_ARGS=(--max-vel "$MAX_VEL")
    vel_desc="限速=${MAX_VEL}m/s"
else
    vel_desc="限速=程序默认"
fi

log "启动选手程序：$SCRIPT（长机=$LEADER 僚机=$FOLLOWER 间距=${SPACING}米 ${vel_desc}）"
docker run -d --name "$C_LEADER" "${COMMON[@]}" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$LEADER" --role leader \
    --teammate "$FOLLOWER" --spacing "$SPACING" "${VEL_ARGS[@]}" "${EXTRA[@]}" >/dev/null
docker run -d --name "$C_FOLLOWER" "${COMMON[@]}" "$IMAGE" \
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
        for c in "$C_LEADER" "$C_FOLLOWER"; do echo "--- $c:" >&2; docker logs --tail 8 "$c" 2>&1 >&2; done
        exit 1
    fi
    sleep 5
done

# ---- 7. 收尾 ----
# 监视自带的"任务结束"判据是给编队飞行写的，任务流程里不一定成立，直接发
# SIGINT 让它存图再退，给几秒写完。
docker exec "$FSNX01" pkill -INT -f monitor.py >/dev/null 2>&1 || true
sleep 6
# 容器以 root 往 /logs 写，宿主机这边属主会是 root，普通用户删不掉——改回来
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
if [ "${EKF_RECORD:-1}" != "0" ] && ls "$LOGDIR/${SCRIPT%.py}_rec_"*.txt >/dev/null 2>&1; then
    echo "EKF录制：runtime_logs/${SCRIPT%.py}_rec_{state,estimator_status}_{NX01,NX02}.txt"
    echo "        （飞机被莫名放到地上时查这个：mode 出现 ALTCTL、或哪位 status_flag 从 true 翻成 false）"
fi
for d in 任务2照片 任务3照片 综合任务照片; do
    [ -d "$LOGDIR/$d" ] && echo "照片：  runtime_logs/$d/ （$(ls -1 "$LOGDIR/$d" | wc -l) 张）"
done
[ "$KEEP" = "1" ] && echo "（--keep：选手容器保留，docker logs $C_LEADER 看完整日志）"
echo "=============================================="
exit "$rc"
