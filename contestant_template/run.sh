#!/usr/bin/env bash
# 一键仿真：主机开机后跑这一个脚本就够了。
#
# 参数就是**要跑的程序文件名**（本目录里的 .py，.py 可省略，可 Tab 补全）：
#
#   ./run.sh formation         # 编队飞行
#   ./run.sh groundfire        # 地面火情：侦查 -> 取物资 -> 投弹 -> 编队返航
#   ./run.sh highrise          # 高层火情：巡检拍摄 -> 协同灭火 -> 编队返回
#   ./run.sh mission           # 三轮连贯：编队 + 两种火情（火情随机、两轮不重复）
#
#   以上四个各有一份"简化版"，加 _lite：formation_lite / groundfire_lite /
#   highrise_lite / mission_lite。行为相同，实现搬进了 SDK（选手代码少一个
#   数量级）。老版一行没动，两版并存。
#
#   ./run.sh                   # 不给就跑 formation.py
#   ./run.sh highrise --keep   # 结束后保留选手容器，便于翻日志
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
# 程序按名字在本目录找，找不到再去 normal/：
#     ./                你自己写的程序 + 四个 *_lite.py
#     ./normal/         详细版四个 + utils.py
# 找到之后，程序**所在的那个目录**被整个挂成容器里的 /workspace，所以同目录
# 的依赖（normal 版的 utils.py、彼此之间的 import）就地解析。
# monitor.py / referee.py 直接从 scripts/ 取。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # 选手目录 = 本脚本所在目录
ROOT="$(cd "$HERE/.." && pwd)"                       # docker_sim 仓库根（仿真设施）

SCRIPT=formation.py
SPACING=4.0
RESTART=1
KEEP=0
TIMEOUT_S=1500
while [ $# -gt 0 ]; do
    case "$1" in
        --spacing)       SPACING="$2"; shift 2 ;;
        --no-restart)    RESTART=0; shift ;;
        --keep)          KEEP=1; shift ;;
        --timeout)       TIMEOUT_S="$2"; shift 2 ;;
        -h|--help)       sed -n '2,26p' "$0"; exit 0 ;;
        -*) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
        *)               SCRIPT="${1%.py}.py"; shift ;;   # 程序文件名，.py 可省
    esac
done

# 航线（世界坐标）。只有 formation.py 吃 --route；两个任务脚本的航点写在自己
# 代码里，传了也没用，所以下面只给编队那一支传。
ROUTE="3,3 3,22 17,22 17,16 10,16 14,14 17,16 17,3"

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

# ---- 2. 定位程序：按名字去源目录找，不保留副本 ----
# 2026-10-01：以前 run.sh 住在单独的"一键仿真"目录里，程序要先同步一份副本
# 过去才能跑——两份真相，同步只是在给它打补丁，而且每次改代码都得来回 cd。
# 现在脚本跟程序待在同一个目录，按名字就地解析。
# 查找顺序（先找到先用）：
#   1. ./        —— 你自己写的程序，和四个 *_lite.py
#   2. ./normal/ —— 详细版四个 + utils.py
# 程序所在的那个目录会被整个挂成容器里的 /workspace，所以同目录的依赖
# （normal 版的 utils.py、彼此之间的 import）就地解析，不用另挂。
for d in "$HERE" "$HERE/normal"; do
    [ -f "$d/$SCRIPT" ] && { SRCDIR="$d"; break; }
done
if [ -z "${SRCDIR:-}" ]; then
    echo "!! 找不到 $SCRIPT。可跑的程序：" >&2
    for d in "$HERE" "$HERE/normal"; do
        # `|| true`：第一个目录（本目录）通常没有 .py，ls 失败会被
        # `set -eo pipefail` 当成致命错误，列举还没开始就退出了。
        ( cd "$d" 2>/dev/null && ls -1 *.py 2>/dev/null \
          | grep -v '^monitor\.py$\|^referee\.py$\|^utils\.py$' \
          | sed "s|^|     ${d#$ROOT/}/|" ) >&2 || true
    done
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
        --out '$MON_OUT' --spacing $SPACING $MON_END > '$MON_LOG' 2>&1
" >/dev/null 2>&1 || echo "（监视没起来，不影响飞行）" >&2

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
# 本目录挂 /workspace。normal 版程序要 import utils/formation/highrise，这些
# 程序所在目录整个挂成 /workspace，同目录的依赖就地解析；
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

log "启动选手程序：$SCRIPT（长机=$LEADER 僚机=$FOLLOWER 间距=${SPACING}米）"
docker run -d --name "$C_LEADER" "${COMMON[@]}" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" --namespace "$LEADER" --role leader \
    --teammate "$FOLLOWER" --spacing "$SPACING" "${EXTRA[@]}" >/dev/null
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
for d in 任务2照片 任务3照片 综合任务照片; do
    [ -d "$LOGDIR/$d" ] && echo "照片：  runtime_logs/$d/ （$(ls -1 "$LOGDIR/$d" | wc -l) 张）"
done
[ "$KEEP" = "1" ] && echo "（--keep：选手容器保留，docker logs $C_LEADER 看完整日志）"
echo "=============================================="
exit "$rc"
