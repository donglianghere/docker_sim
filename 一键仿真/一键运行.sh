#!/usr/bin/env bash
# 一键仿真：主机开机后跑这一个脚本就够了。
#
#   ./一键运行.sh              # 默认跑编队飞行
#   ./一键运行.sh 任务2         # 地面火情：侦查 -> 取物资 -> 投弹 -> 编队返航
#   ./一键运行.sh 任务3         # 高层火情：巡检拍摄 -> 协同灭火 -> 编队返回
#   ./一键运行.sh 任务3 --keep  # 结束后保留选手容器，便于翻日志
#
# 它做这些事（按顺序）：
#   1. 检查 docker / 镜像 / X11
#   2. 起仿真容器（Gazebo + RViz 由 .env 的 USE_GAZEBO_GUI/USE_RVIZ 自动拉起）
#   3. 等两机就绪（ROS 节点 + PX4 自检都过）
#   4. 起监视窗口（实时俯视轨迹 / 高度 / 间距 + 飞行统计）
#   5. 跑选手程序（长机僚机各一个容器）
#   6. 收尾：让监视存图、修正日志属主、打印结果
#
# 这个目录里的 5 个文件就是实际跑起来的那 5 个：三个任务程序 + 监视程序 +
# 本脚本。任务程序还要用到 contestant_template 下的三个依赖模块
# （任务工具 / 地面火情搜索示例 / 高楼火情绕飞版示例），脚本把本目录挂成
# /workspace（优先）、contestant_template 挂成 /deps 补齐，所以**跑的是本
# 目录这几份**，不是 contestant_template 里的。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"          # docker_sim 仓库根

SCENE=编队
SPACING=4.0
RESTART=1
KEEP=0
TIMEOUT_S=1500
while [ $# -gt 0 ]; do
    case "$1" in
        编队|formation)  SCENE=编队;  shift ;;
        任务2|task2)     SCENE=任务2; shift ;;
        任务3|task3)     SCENE=任务3; shift ;;
        --spacing)       SPACING="$2"; shift 2 ;;
        --no-restart)    RESTART=0; shift ;;
        --keep)          KEEP=1; shift ;;
        --timeout)       TIMEOUT_S="$2"; shift 2 ;;
        -h|--help)       sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
    esac
done

case "$SCENE" in
    编队)  SCRIPT=编队飞行示例.py ;;
    任务2) SCRIPT=任务2单项测试.py ;;
    任务3) SCRIPT=任务3单项测试.py ;;
esac

# 航线（世界坐标）。只有编队飞行示例吃 --route；两个任务脚本的航点写在自己
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
cleanup() { [ "$KEEP" = "1" ] && return 0; docker rm -f "$C_LEADER" "$C_FOLLOWER" >/dev/null 2>&1 || true; }
trap cleanup EXIT

echo "=============================================="
echo " 一键仿真：$SCENE（$SCRIPT）"
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
for f in "$SCRIPT" 编队监视.py; do
    [ -f "$HERE/$f" ] || { echo "!! 本目录缺 $f !!" >&2; exit 1; }
done

# X11：容器里的 Gazebo/RViz/监视窗口都要往宿主机的 X server 上画。
# DISPLAY 没设时给个默认值——从 tty 或 ssh 进来跑的话它可能是空的。
export DISPLAY="${DISPLAY:-:1}"
if command -v xhost >/dev/null; then
    xhost +local: >/dev/null 2>&1 || echo "（xhost 授权没成功，窗口可能弹不出来）" >&2
fi
log "DISPLAY=$DISPLAY"

# ---- 2. 静态自检：语法之外，还查"调用了不存在的函数"这类运行时才炸的错 ----
if [ -f "$ROOT/scripts/check_python_static.py" ]; then
    if ! python3 "$ROOT/scripts/check_python_static.py" "$HERE"/*.py; then
        echo "!! 本目录的程序静态自检不通过，先修好再飞 !!" >&2
        exit 1
    fi
fi
# 顺带提醒：本目录是 contestant_template 的副本，两边改歪了要知道
for f in 编队飞行示例.py 任务2单项测试.py 任务3单项测试.py; do
    src="$ROOT/contestant_template/$f"
    [ -f "$src" ] || continue
    cmp -s "$src" "$HERE/$f" || echo "（注意：$f 跟 contestant_template 里的那份已经不一样了，本次跑的是本目录这份）" >&2
done
cmp -s "$ROOT/scripts/编队监视.py" "$HERE/编队监视.py" || \
    echo "（注意：编队监视.py 跟 scripts/ 里的那份已经不一样了，本次跑的是本目录这份）" >&2

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
MON_OUT="/logs/${SCRIPT%.py}_formation.png"
MON_LOG="/logs/${SCRIPT%.py}_monitor.log"
log "启动监视窗口（报告将存到 runtime_logs/$(basename "$MON_OUT")）"
docker cp "$HERE/编队监视.py" "$FSNX01:/tmp/编队监视.py" >/dev/null 2>&1 || true
docker exec -d -e DISPLAY="$DISPLAY" "$FSNX01" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0 \
           RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
           CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml
    python3 -u /tmp/编队监视.py --layout '$LAYOUT' --route '$ROUTE' \
        --out '$MON_OUT' --spacing $SPACING > '$MON_LOG' 2>&1
" >/dev/null 2>&1 || echo "（监视没起来，不影响飞行）" >&2

# ---- 6. 跑选手程序 ----
# 本目录挂 /workspace（优先），contestant_template 挂 /deps 补三个依赖模块；
# /etc/localtime 挂进去，照片时间戳和统计里的时刻才是本地时间（否则是 UTC）。
# PYTHONPATH 必须**在镜像原值后面追加**，不能直接覆盖：镜像 ENV 里带着
# /opt/quadrotor_msgs_install/...，`-e PYTHONPATH=/workspace:/deps` 会把它整个
# 顶掉，飞起来就是 ModuleNotFoundError: No module named 'quadrotor_msgs'。
BASE_PYPATH="$(docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$IMAGE" \
               | sed -n 's/^PYTHONPATH=//p' | head -1)"
PYPATH="/workspace:/deps${BASE_PYPATH:+:$BASE_PYPATH}"
PYPATH="${PYPATH%:}"      # 镜像原值自带尾部冒号，空条目会把 CWD 也塞进 sys.path
COMMON=(--network host
        -v /etc/localtime:/etc/localtime:ro
        -v "$HERE:/workspace"
        -v "$ROOT/contestant_template:/deps:ro"
        -v "$LOGDIR:/logs"
        -e PYTHONPATH="$PYPATH")
EXTRA=()
[ "$SCENE" = "编队" ] && EXTRA=(--route "$ROUTE")

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
docker exec "$FSNX01" pkill -INT -f 编队监视 >/dev/null 2>&1 || true
sleep 6
# 容器以 root 往 /logs 写，宿主机这边属主会是 root，普通用户删不掉——改回来
docker run --rm -v "$LOGDIR:/logs" --entrypoint chown "$IMAGE" \
    -R "$(id -u):$(id -g)" /logs >/dev/null 2>&1 || true

echo
echo "================ $SCENE 结果 ================"
rc=0
for c in "$C_LEADER" "$C_FOLLOWER"; do
    code="$(docker inspect -f '{{.State.ExitCode}}' "$c" 2>/dev/null || echo '?')"
    echo "--- $c 退出码 $code"
    [ "$code" = "0" ] || rc=1
    docker logs "$c" 2>&1 | grep -E 'Traceback|Error|错误|失败' | tail -5 || true
done
echo
echo "报告图：runtime_logs/$(basename "$MON_OUT")"
[ -d "$LOGDIR/任务3照片" ] && echo "照片：  runtime_logs/任务3照片/"
[ "$KEEP" = "1" ] && echo "（--keep：选手容器保留，docker logs $C_LEADER 看完整日志）"
echo "=============================================="
exit "$rc"
