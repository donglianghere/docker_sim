#!/usr/bin/env bash
# 跑 c5_夹爪往复.py（张开/闭合 3 次）。在地面站的选手容器里跑，真机域 20。
#
#   ./c5_夹爪往复.sh
#
# ⚠️ 爪子会连续动 6 下，确认附近没有手、爪子里没夹着东西。
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
IP=192.168.2.102          # NX02（夹具在任务机上）

ping -c1 -W2 "$IP" >/dev/null 2>&1 || { echo "!! 飞机 $IP 不可达" >&2; exit 1; }
st="$(curl -s --noproxy '*' --max-time 8 "http://$IP:8890/status" 2>/dev/null || true)"
echo "$st" | grep -q '"flight_up": true' || {
    echo "!! NX02 的 flight-stack 没起——没有 mavros 就发不出执行器指令" >&2
    echo "   起它：地面站界面点按钮，或 curl -s --noproxy '*' -X POST \\" >&2
    echo "     -H 'Content-Type: application/json' \\" >&2
    echo "     -d '{\"stack\":\"flight\",\"action\":\"up\",\"confirm\":true}' http://$IP:8890/stack" >&2
    exit 1; }

# shellcheck source=/dev/null
source "$ROOT/scripts/contestant_network.sh"
contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" \
    || { echo "!! 真机网络参数生成失败" >&2; exit 1; }

exec timeout 120 docker run --rm --network host \
    -v "$HERE":/workspace:ro -v /etc/localtime:/etc/localtime:ro \
    "${CONTESTANT_NET_ARGS[@]}" "contestant-sdk:latest" \
    bash -lc 'source /opt/ros/humble/setup.bash && python3 -u /workspace/c5_夹爪往复.py'
