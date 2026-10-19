#!/usr/bin/env bash
# 地面检查 4：三路执行器（MAIN5 发射器 + MAIN7/MAIN9 双爪）。**不起飞、不解锁。**
#
#   ./c4_执行器.sh --open               # 打开爪子（两爪 -> 2100）
#   ./c4_执行器.sh --grip               # 抓取（两爪 -> 900）
#   ./c4_执行器.sh --params             # 只核飞控参数，一下都不动
#   ./c4_执行器.sh --shoot              # 发射（推到发射位再复位）
#   ./c4_执行器.sh --launcher 1500      # 发射机构停在指定 PWM（试行程）
#   ./c4_执行器.sh --ns NX01 --params   # 换机号
#
# 2026-10-15 夹爪改装：爪子从 MAIN7+MAIN10 挪到 MAIN7+MAIN9，开合方向同时反转，
# 2026-10-18 起：爪子两机都是 **1000~1950**（用户给定），发射机构**两机不同**
# —— NX01 是 900~2100（从那架飞控读回）、NX02 是 800~2200。
# 方向两机一样：爪子下端=抓紧、发射器上端=发射位。**--params 要两机分别跑。**
# PWM 值和核哪些参数都来自 SDK 的 SERVO_CONFIG，这个脚本里不写死。
#
# 在地面站的选手容器里跑（真机域 20），不需要 ssh 上飞机。动作和参数核对都在
# 选手 SDK 里（check_actuators/grip/shoot），c4_执行器.py 只做命令行分发。
#
# 爪子/发射器的往复验证另有两个极简程序：c5_夹爪往复.sh、c6_发射器.sh。
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
IMAGE=contestant-sdk:latest

NS=NX02
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --ns) NS="$2"; shift 2 ;;
        -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done
case "$NS" in
    NX01) IP=192.168.2.101 ;;
    NX02) IP=192.168.2.102 ;;
    *) echo "!! 未知机号：$NS" >&2; exit 2 ;;
esac

ping -c1 -W2 "$IP" >/dev/null 2>&1 || { echo "!! 飞机 $IP 不可达" >&2; exit 1; }
st="$(curl -s --noproxy '*' --max-time 8 "http://$IP:8890/status" 2>/dev/null || true)"
fu="$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("flight_up"))' 2>/dev/null || true)"
if [ "$fu" != "True" ]; then
    echo "!! $NS($IP) 的 flight-stack 没起（flight_up=$fu）——没有 mavros 就发不出执行器指令" >&2
    echo "   起它：地面站界面点按钮，或" >&2
    echo "     curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \\" >&2
    echo "          -d '{\"stack\":\"flight\",\"action\":\"up\",\"confirm\":true}' \\" >&2
    echo "          http://$IP:8890/stack" >&2
    exit 1
fi

# shellcheck source=/dev/null
source "$ROOT/scripts/contestant_network.sh"
contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" \
    || { echo "!! 真机网络参数生成失败" >&2; exit 1; }
echo "网络：$CONTESTANT_NET_DESC"

exec timeout 120 docker run --rm --network host \
    -v "$HERE":/workspace:ro -v /etc/localtime:/etc/localtime:ro \
    "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    bash -lc "source /opt/ros/humble/setup.bash && \
              python3 -u /workspace/c4_执行器.py --ns $NS ${ARGS[*]}"
