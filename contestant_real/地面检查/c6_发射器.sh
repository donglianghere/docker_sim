#!/usr/bin/env bash
# 跑 c6_发射器.py（逐档走行程，验证方向）。在地面站的选手容器里跑，真机域 20。
#
#   ./c6_发射器.sh [NX01|NX02]
#
# ⚠️ 确认发射机构里没有装弹、机构前方无人。
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
NS="${1:-NX02}"           # 机号，默认 NX02
case "$NS" in
    NX01) IP=192.168.2.101 ;;
    NX02) IP=192.168.2.102 ;;
    *) echo "!! 未知机号：$NS" >&2; exit 2 ;;
esac

ping -c1 -W2 "$IP" >/dev/null 2>&1 || { echo "!! 飞机 $IP 不可达" >&2; exit 1; }
st="$(curl -s --noproxy '*' --max-time 8 "http://$IP:8890/status" 2>/dev/null || true)"
echo "$st" | grep -q '"flight_up": true' || {
    echo "!! $NS 的 flight-stack 没起——没有 mavros 就发不出执行器指令" >&2
    echo "   起它：地面站界面点按钮，或 curl -s --noproxy '*' -X POST \\" >&2
    echo "     -H 'Content-Type: application/json' \\" >&2
    echo "     -d '{\"stack\":\"flight\",\"action\":\"up\",\"confirm\":true}' http://$IP:8890/stack" >&2
    exit 1; }

# shellcheck source=/dev/null
source "$ROOT/scripts/contestant_network.sh"
contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" \
    || { echo "!! 真机网络参数生成失败" >&2; exit 1; }

exec timeout 180 docker run --rm --network host \
    -v "$HERE":/workspace:ro -v /etc/localtime:/etc/localtime:ro \
    "${CONTESTANT_NET_ARGS[@]}" "contestant-sdk:latest" \
    bash -lc "source /opt/ros/humble/setup.bash && python3 -u /workspace/c6_发射器.py $NS"
