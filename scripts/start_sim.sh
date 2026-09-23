#!/usr/bin/env bash
# 启动仿真机队，并**验证相机真的在出图**之后才返回。所有仿真测试都应该用
# 这个脚本启动，不要直接 docker compose up。
#
#   ./scripts/start_sim.sh                 # 重启仿真，等就绪，验证相机
#   ./scripts/start_sim.sh --gui           # 同时开 Gazebo 图形界面
#   ./scripts/start_sim.sh --no-restart    # 沿用当前在跑的仿真，只做验证
#
# 为什么要有这个脚本（2026-09-23）：之前有两轮测试"搜索不到地面火点"，排查
# 半天发现是**宿主机 X 授权掉了**——容器里是 root，`xhost` 只授权了宿主机
# 用户时，gzserver 连不上 X server，相机需要的 OpenGL 渲染起不来，相机话题
# 一个发布者都没有。而雷达/IMU/仿真时间全都正常，飞机照常起飞、飞完航线、
# 降落，日志里毫无异常，只是"什么都没看见"。同一个坑昨天没踩到只是因为昨天
# 授权还在（重启宿主机/重新登录桌面就会掉），这种运气不该由人每次去记。
#
# 所以这里把三件事固定下来：①启动前先 xhost 授权；②启动后逐路相机确认
# **检测节点真的收到帧**（不是"话题存在"，是端到端有数据）；③验证失败时
# 直接非零退出并打印该查什么，不让测试带着坏环境跑下去。
set -eo pipefail

cd "$(dirname "$0")/.."

RESTART=1
GUI=0
READY_TIMEOUT_S=240

while [ $# -gt 0 ]; do
    case "$1" in
        --no-restart) RESTART=0; shift ;;
        --gui)        GUI=1; shift ;;
        --timeout)    READY_TIMEOUT_S="$2"; shift 2 ;;
        -h|--help)    sed -n '2,9p' "$0"; exit 0 ;;
        *) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
    esac
done

AGENTS="NX01 NX02"
CAMERAS="down front"
log() { echo "[$(date +%H:%M:%S)] $*"; }

# ---- 1. X 授权：容器里的 gzserver(相机渲染)/gzclient/rviz2 都要连宿主机 X ----
# 只能由启动这个 X 会话的宿主机用户执行（容器里的 root 不行）。没有 DISPLAY
# （纯 SSH）时授权会失败，此时相机也一定起不来，直接当错误退出，别往下飞。
if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
    if xhost +local:root >/dev/null 2>&1; then
        log "X 授权已加（xhost +local:root）"
    else
        echo "!! xhost 授权失败，相机会起不来——确认这个终端在图形桌面里、DISPLAY=$DISPLAY 正确 !!" >&2
        exit 1
    fi
else
    echo "!! 没有 DISPLAY 或没装 xhost：gzserver 无法渲染，相机不会出图（纯SSH环境跑不了带相机的仿真） !!" >&2
    exit 1
fi

# ---- 2. 起容器 ----
if [ "$RESTART" = "1" ]; then
    log "重启仿真容器（GUI=$GUI）…"
    docker ps --filter ancestor=contestant-sdk:latest --format '{{.Names}}' \
        | grep -v sound | xargs -r docker rm -f >/dev/null 2>&1 || true
    docker compose down --timeout 20 >/dev/null 2>&1 || true
    USE_GAZEBO_GUI=$([ "$GUI" = "1" ] && echo true || echo false) \
        docker compose up -d >/dev/null
fi

# ---- 3. 等就绪：PX4 自检过 + 飞控栈节点起来 ----
# 等 PX4 自己报 "Ready for takeoff"，不是 sleep 固定秒数：节点起来时 PX4 的
# 起飞前自检往往还没过，这时发起飞会被 ARM rejected 拒掉。
# 注意不能写成 `docker logs ... | grep -q`：pipefail 下 grep -q 提前退出会
# 让 docker logs 吃 SIGPIPE，整条管道被判失败。
log_has() {
    local out
    out="$(docker logs "$1" 2>&1 || true)"
    case "$out" in *"$2"*) return 0 ;; *) return 1 ;; esac
}

log "等两机就绪（PX4 自检 + 飞控栈节点）…"
DEADLINE=$(( $(date +%s) + READY_TIMEOUT_S ))
while :; do
    ok=1
    for ns in $AGENTS; do
        docker exec docker_sim-sim-world-1 grep -q "Ready for takeoff" "/tmp/px4_${ns}.log" \
            2>/dev/null || ok=0
    done
    for c in nx01 nx02; do
        log_has "docker_sim-flight-stack-$c-1" 'formation_follower_node就绪' || ok=0
    done
    [ "$ok" = "1" ] && break
    if [ "$(date +%s)" -ge "$DEADLINE" ]; then
        echo "!! ${READY_TIMEOUT_S}秒内没等到两机就绪，看 docker logs docker_sim-sim-world-1 !!" >&2
        exit 1
    fi
    sleep 5
done
log "两机已就绪"

# ---- 4. 验证相机端到端出图（这一步是这个脚本存在的理由）----
# 判据用检测节点自己每 5 秒打的"最近5秒处理 N 帧"：它在链路末端，N>0 说明
# 渲染、DDS、接收缓冲整条路都是通的。等两个统计周期，避免刚启动那一版还是 0。
log "验证四路相机出图…"
sleep 12
bad=""
for c in nx01 nx02; do
    for cam in $CAMERAS; do
        line="$(docker logs --since 30s "docker_sim-flight-stack-$c-1" 2>&1 \
                | grep "camera_tag_detect_${cam}.*处理" | tail -1 || true)"
        frames="$(echo "$line" | sed -nE 's/.*处理 ([0-9]+) 帧.*/\1/p')"
        if [ -z "$frames" ] || [ "$frames" -eq 0 ]; then
            bad="$bad ${c}/${cam}"
        else
            log "  ${c}/${cam}: ${frames} 帧/5秒"
        fi
    done
done

if [ -n "$bad" ]; then
    echo "!! 这几路相机没有出图：$bad" >&2
    echo "!! 按这个顺序查：" >&2
    echo "!!  1) gzserver 还活着吗： docker logs --tail 20 docker_sim-sim-world-1" >&2
    echo "!!     （用 gz model -p 之类的 gz 命令查真值会把 gzserver 打崩，别用，改读 /plug/model_states_plug）" >&2
    echo "!!  2) 相机话题有没有发布者： ros2 topic info -v /NX01/NX01_down_camera/image_raw" >&2
    echo "!!     Publisher count=0 就是渲染没起来（X 授权/显卡），不是 DDS 问题" >&2
    echo "!!  3) 有发布者但收不到帧： cat /proc/sys/net/core/rmem_max 应为 33554432（见 /etc/sysctl.d/60-ros2-dds-buffers.conf）" >&2
    exit 1
fi

# ---- 5. 打印这次的场景坐标，省得去猜/去用 gz 命令查 ----
scenario="$(docker logs docker_sim-sim-world-1 2>&1 | grep -o "NX01->.*checkpoint已重置" | tail -1 || true)"
[ -n "$scenario" ] && log "本次场景：$scenario"
log "仿真就绪，相机正常"
