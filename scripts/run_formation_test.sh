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

ROUTE="7,-10 7,10 -8,10 -8,-10"
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
log "等两机就绪…"
READY_DEADLINE=$(( $(date +%s) + 180 ))
while :; do
    ok=1
    for c in nx01 nx02; do
        docker logs "docker_sim-flight-stack-$c-1" 2>&1 \
            | grep -q 'formation_follower_node就绪' || ok=0
    done
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
        docker logs "$c" 2>&1 | grep -q '编队飞行结束' && done_cnt=$((done_cnt + 1))
    done
    [ "$done_cnt" = "2" ] && { log "两机都已完成"; break; }

    # 容器退了但没打印"编队飞行结束"=异常终止
    for c in fm_leader fm_follower; do
        if ! docker ps --format '{{.Names}}' | grep -qx "$c"; then
            if ! docker logs "$c" 2>&1 | grep -q '编队飞行结束'; then
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
[ "$KEEP" = "1" ] && echo "（--keep：选手容器保留，用 docker logs fm_leader / fm_follower 看完整日志）"
