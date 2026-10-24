#!/usr/bin/env bash
# 一键仿真：起仿真容器 -> 等两机就绪 -> 起监视窗口 -> 跑选手程序 -> 收尾。
#
# 用法：./run.sh [程序名] [选项]      程序名 = 本目录下的 .py（.py 可省、可 Tab 补全）
#   ./run.sh formation_slim          编队飞行（不给程序名时的默认）
#   ./run.sh groundfire_slim         地面火情：侦查 -> 取物资 -> 投弹 -> 编队返航
#   ./run.sh highrise_slim           高层火情：巡检拍摄 -> 协同灭火 -> 编队返回
#   ./run.sh mission_slim            三轮连贯：编队 + 两种火情（火情随机、两轮不重复）
#
# 选项：
#   --spacing N     编队目标间距（米），跟 --min-sep 一起调
#   --min-sep N     两机直线距离下限（米），低于它僚机参考点冻结
#   --max-vel N     经规划器航段的巡航限速（m/s），不管直飞
#   --no-restart    不重起仿真容器
#   --keep          结束后保留选手容器，便于翻日志
#   --timeout N     选手程序超时秒数
#
# 环境变量：SOUND_LIGHT_IGNORE=1 跳过声光串口检查（本轮没有声光播报）。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # 选手目录 = 本脚本所在目录
ROOT="$(cd "$HERE/.." && pwd)"                       # docker_sim 仓库根（仿真设施）

SCRIPT=formation_slim.py   # 2026-10-14：原 formation.py 已删
# 编队目标间距（米）：传给选手程序覆盖 SDK 默认值，监视器的目标线也用它。
SPACING=5.0
# 经规划器航段的巡航限速（m/s）。不管直飞——那条走 SDK 的 DIRECT_CRUISE_MPS。
MAX_VEL=1.0
# 两机直线距离下限（米），低于它僚机参考点冻结。留空=用 .env 的值；调 --spacing 要一起调。
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
        -h|--help)       sed -n '2,18p' "$0"; exit 0 ;;
        -*) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
        *)               SCRIPT="${1%.py}.py"; shift ;;   # 程序文件名，.py 可省
    esac
done

# 监视器算"沿航线间距"用的航线，必须跟长机实际带队跑的那条逐点一致（含闭合段）。
# 2026-10-23 起由 scripts/route_for_monitor.py 从 venue.py 现算，**不再在这里
# 抄第二份坐标**——以前抄件改了 venue 忘同步，间距会读成整条航线长（实测 67 米、
# 占 38% 样本）；同日核对发现 run_real.sh 那份确实已经漂了 4 个点。
# 算不出来就不传 --route：少一条间距曲线而已，不该让整个架次飞不了。
ROUTE="$(python3 "$ROOT/scripts/route_for_monitor.py" "$HERE" "$SCRIPT" 2>&1)" || {
    echo "!! 航线算不出来（$ROUTE），本架次监视器不画沿航线间距" >&2
    ROUTE=""
}

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
die() { echo "!! $*" >&2; exit 1; }

# ---- 互斥锁：同一时刻只允许一个 run.sh 操作这套仿真 ----
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

# 容器里的 Gazebo/RViz/监视窗口都往宿主机的 X server 上画。
export DISPLAY="${DISPLAY:-:1}"
if command -v xhost >/dev/null; then
    xhost +local: >/dev/null 2>&1 || echo "（xhost 授权没成功，窗口可能弹不出来）" >&2
fi
log "DISPLAY=$DISPLAY"

# ---- 1.5 真机残留检查（2026-10-03 新增）----
# ---- 共享栈必起（2026-10-03 用户要求）----
source "$ROOT/scripts/ensure_stacks.sh"
check_sound_light_port
ensure_sound_light sim

"$ROOT/scripts/check_env.sh" sim || { echo "!! 环境检查未通过（见上），处理后再跑" >&2; exit 1; }

# ---- 2. 定位程序：按名字去源目录找，不保留副本 ----
SRCDIR=""
[ -f "$HERE/$SCRIPT" ] && SRCDIR="$HERE"
if [ -z "$SRCDIR" ]; then
    echo "!! 找不到 $SCRIPT。可跑的程序：" >&2
    # `|| true`：目录里一个 .py 都没有时 ls 失败，pipefail 下会直接退出。
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
# 装了 pyright 就顺带做一次类型检查；没装就跳过，它不是飞行的硬依赖。
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
# 不能写成 `docker logs | grep -q`：pipefail 下 grep 先退出会把 docker logs 打成 141。
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

# Gazebo / RViz 由 sim-world 容器自己拉起，这里只确认，没起来不致命。
sleep 2
gz_n="$(docker exec "$SIMWORLD" bash -lc 'pgrep -c gzclient || true' 2>/dev/null | tr -d '\r')"
rv_n="$(docker exec "$SIMWORLD" bash -lc 'pgrep -c rviz2 || true' 2>/dev/null | tr -d '\r')"
log "Gazebo 界面 ${gz_n:-0} 个、RViz ${rv_n:-0} 个"
[ "${gz_n:-0}" = "0" ] && echo "（Gazebo 界面没起：检查 .env 的 USE_GAZEBO_GUI=true 和 xhost 授权）" >&2
[ "${rv_n:-0}" = "0" ] && echo "（RViz 没起：检查 .env 的 USE_RVIZ=true）" >&2

# ---- 5. 监视窗口 ----
MON_END=""
# mission 是多轮流程，收尾判据要用"两机都降落"。
case "$SCRIPT" in mission*.py) MON_END="--end-on-all-land" ;; esac
# ---- --min-sep：把安全下限设到两机的 formation_follower_node 上 ----
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
    # 等 DDS 收敛再起选手程序：紧接着起会报 takeoff:set_height 超时。
    sleep 5
fi

# 报告图的名字就是"时间 + 飞行报告"，每趟一份、不互相覆盖。
MON_OUT="/logs/$(date "+%Y-%m-%d %H%M") 飞行报告.png"
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
# 只有老版 formation.py 吃 --route；其余程序的航点写在自己代码里。
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
docker exec "$FSNX01" pkill -INT -f monitor.py >/dev/null 2>&1 || true
sleep 6
# 容器以 root 往 /logs 写，属主改回当前用户，否则普通用户删不掉。
docker run --rm -v "$LOGDIR:/logs" --entrypoint chown "$IMAGE" \
    -R "$(id -u):$(id -g)" /logs >/dev/null 2>&1 || true

echo
echo "================ $SCRIPT 结果 ================"
rc=0
for c in "$C_LEADER" "$C_FOLLOWER"; do
    code="$(docker inspect -f '{{.State.ExitCode}}' "$c" 2>/dev/null || echo '?')"
    echo "--- $c 退出码 $code"
    [ "$code" = "0" ] || rc=1
    # 先落盘再摘要：下面 cleanup 会 docker rm -f，不存下来完整日志就没了。
    docker logs "$c" > "$LOGDIR/${SCRIPT%.py}_$c.log" 2>&1 || true
    docker logs "$c" 2>&1 | grep -E 'Traceback|Error|错误|失败' | tail -5 || true
done
echo
echo "报告图：runtime_logs/$(basename "$MON_OUT")"
echo "程序日志：runtime_logs/${SCRIPT%.py}_{$C_LEADER,$C_FOLLOWER}.log"
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
