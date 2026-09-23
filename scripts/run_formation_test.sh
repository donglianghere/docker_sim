#!/usr/bin/env bash
# 一键跑双机编队飞行测试：重启仿真 -> 等两机就绪 -> 起两个选手程序 ->
# 等飞完 -> 打印结果 -> 收尾。
#
#   ./scripts/run_formation_test.sh
#   ./scripts/run_formation_test.sh --route "5,-8 5,8 -5,8" --spacing 4.0
#   ./scripts/run_formation_test.sh --no-restart      # 沿用当前已经在跑的仿真
#   ./scripts/run_formation_test.sh --keep            # 结束后不删选手容器，便于翻日志
#
# 跑的是 contestant_template/编队飞行示例.py，两架飞机各起一个容器、
# 跑同一份代码，只有 --role 不同。
set -eo pipefail

cd "$(dirname "$0")/.."

ROUTE="7,-9.5 7,9.5 -7,9.5 -7,-9.5"
SPACING=3.5
RESTART=1
KEEP=0
# 超时按最坏情况估：僚机起飞最多 preflight 150s + 起飞判定 60s，
# 加上航线本身，给 15 分钟。
TIMEOUT_S=900

while [ $# -gt 0 ]; do
    case "$1" in
        --route)      ROUTE="$2"; shift 2 ;;
        --spacing)    SPACING="$2"; shift 2 ;;
        --no-restart) RESTART=0; shift ;;
        --keep)       KEEP=1; shift ;;
        --timeout)    TIMEOUT_S="$2"; shift 2 ;;
        -h|--help)    sed -n '2,11p' "$0"; exit 0 ;;
        *) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
    esac
done

LEADER=NX01
FOLLOWER=NX02
IMAGE=contestant-sdk:latest
WORKDIR="$PWD/contestant_template"
SCRIPT="编队飞行示例.py"

log() { echo "[$(date +%H:%M:%S)] $*"; }

cleanup() {
    [ "$KEEP" = "1" ] && return 0
    docker rm -f fm_leader fm_follower >/dev/null 2>&1 || true
}
trap cleanup EXIT

# ---- 前置检查：镜像在不在 ----
for img in "$IMAGE" flight-stack:latest sim-world:latest; do
    if ! docker image inspect "$img" >/dev/null 2>&1; then
        echo "!! 镜像 $img 不存在，先 build（见 docker-compose.yml 顶部的 build 说明） !!" >&2
        exit 1
    fi
done
[ -f "$WORKDIR/$SCRIPT" ] || { echo "!! 找不到 $WORKDIR/$SCRIPT !!" >&2; exit 1; }

docker rm -f fm_leader fm_follower >/dev/null 2>&1 || true

# ---- 1. 重启仿真，拿一个干净的初始位姿 ----
if [ "$RESTART" = "1" ]; then
    log "重启仿真容器…"
    docker compose down --timeout 20 >/dev/null 2>&1 || true
    docker compose up -d >/dev/null
fi

# ---- 2. 等两机的编队节点就绪 ----
# 等这条日志而不是 sleep 固定秒数：仿真启动耗时随机器负载变化很大。

# PX4 自己报"可以起飞"才算就绪。只等 ROS 节点起来是不够的——节点起来时
# PX4 的起飞前自检（ekf2 收敛、电源检查）往往还没过，这时候发起飞指令会被
# `ARM rejected by PX4!` 拒掉，只能盲等重试。PX4 在自己的日志里明说了这件
# 事（`INFO [commander] Ready for takeoff!`），直接等它。
#
# 为什么读日志文件而不是订 ROS 话题：mavros 的 statustext 转发通路指望不上
# ——实测从 compose up 开始录 150 秒，`mavros/statustext/recv` 一条都没有，
# 因为 mavros 建立 MAVLink 连接时 PX4 早就把那条消息打完了，STATUSTEXT 不
# 补发。脚本本来就在跑 docker compose、本来就是仿真侧的测试工具，读仿真
# 容器里的 PX4 日志没有任何耦合问题；机载代码一行都不碰这个。

# 注意：这里**不能**写成 `docker logs ... | grep -q PATTERN`。
# `set -o pipefail` 下，grep -q 一匹配上就退出，docker logs 还在往管道写、
# 被 SIGPIPE 杀掉（141），整条管道被判为失败——匹配成功反而当成失败，日志
# 越大越必然触发。所以先读进变量再用 case 匹配，不走管道。
log_has() {   # $1=容器名 $2=要找的字符串
    local out
    out="$(docker logs "$1" 2>&1 || true)"
    case "$out" in
        *"$2"*) return 0 ;;
        *)      return 1 ;;
    esac
}

px4_ready() {
    for f in /tmp/px4_NX01.log /tmp/px4_NX02.log; do
        docker exec docker_sim-sim-world-1 grep -q "Ready for takeoff" "$f" \
            2>/dev/null || return 1
    done
    return 0
}

log "等两机就绪（ROS节点 + PX4自检）…"
READY_DEADLINE=$(( $(date +%s) + 180 ))
while :; do
    ok=1
    for c in nx01 nx02; do
        log_has "docker_sim-flight-stack-$c-1" 'formation_follower_node就绪' || ok=0
    done
    px4_ready || ok=0
    [ "$ok" = "1" ] && break
    if [ "$(date +%s)" -ge "$READY_DEADLINE" ]; then
        echo "!! 等仿真就绪超过180秒，检查 docker compose logs !!" >&2
        exit 1
    fi
    sleep 5
done
log "两机就绪"

# ---- 3. 起两个选手程序 ----
log "启动选手程序（长机=$LEADER 航线=\"$ROUTE\"，僚机=$FOLLOWER 间距=${SPACING}米）"
docker run -d --name fm_leader --network host -v "$WORKDIR:/workspace" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" \
    --namespace "$LEADER" --role leader --teammate "$FOLLOWER" \
    --route "$ROUTE" --spacing "$SPACING" >/dev/null
docker run -d --name fm_follower --network host -v "$WORKDIR:/workspace" "$IMAGE" \
    python3 -u "/workspace/$SCRIPT" \
    --namespace "$FOLLOWER" --role follower --teammate "$LEADER" \
    --spacing "$SPACING" >/dev/null

# ---- 4. 等两边都报"编队飞行结束"；任一容器异常退出就停下来报错 ----
DEADLINE=$(( $(date +%s) + TIMEOUT_S ))
while :; do
    done_cnt=0
    for c in fm_leader fm_follower; do
        log_has "$c" '编队飞行结束' && done_cnt=$((done_cnt + 1))
    done
    [ "$done_cnt" = "2" ] && { log "两机都已完成"; break; }

    # 容器退了但没打印"编队飞行结束"=异常终止
    for c in fm_leader fm_follower; do
        if ! docker ps --format '{{.Names}}' | grep -qx "$c"; then
            if ! log_has "$c" '编队飞行结束'; then
                echo ""
                echo "!! $c 异常退出，末尾日志： !!" >&2
                docker logs --tail 15 "$c" 2>&1 >&2
                exit 1
            fi
        fi
    done

    if [ "$(date +%s)" -ge "$DEADLINE" ]; then
        echo "!! 超过 ${TIMEOUT_S} 秒仍未飞完 !!" >&2
        for c in fm_leader fm_follower; do
            echo "--- $c 末尾日志:" >&2
            docker logs --tail 8 "$c" 2>&1 >&2
        done
        exit 1
    fi
    sleep 5
done

# ---- 5. 结果摘要 ----
echo ""
echo "================ 编队测试结果 ================"
for c in fm_leader fm_follower; do
    echo "--- $c"
    docker logs "$c" 2>&1 | grep -E '朝向模式|就位待命|已被.*确认|航点 1/|回起飞点|降落完成' || true
done
echo ""
echo "僚机沿轨迹间距（track阶段最后几次反馈，应当稳定且不小于 ${SPACING} 米）："
docker logs fm_follower 2>&1 | grep 'track阶段' | tail -3 || echo "  （没有 track 阶段反馈）"
echo "=============================================="
# 注意别写成 `[ cond ] && echo ...`：这是脚本最后一条命令，条件为假时它的
# 退出码1会成为整个脚本的退出码——飞行明明成功，调用方却看到失败。
if [ "$KEEP" = "1" ]; then
    echo "（--keep：选手容器保留，用 docker logs fm_leader / fm_follower 看完整日志）"
fi
exit 0
