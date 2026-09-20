#!/usr/bin/env bash
# 专项排查"雷达网口反复松动/断线"：每秒轮询一次
# `ip -o link show ${LIDAR_IFACE}`的operstate（UP/DOWN/NO-CARRIER等），
# 状态发生变化才写一行，避免刷屏——只要出现过至少一次DOWN/NO-CARRIER，
# 就实锤了"物理层不稳"，不需要等到雷达数据完全断流才后知后觉。
#
# 背景：2026-09-08曾经查到过enP8p1s0处于NO-CARRIER（网线没插好/对端没
# 通电的明确信号），这次专项验证是不是反复出现（一次性的松动 vs
# 间歇性接触不良，处理方式不一样）。
#
# 用法（在真机host上直接跑，不需要进容器——网口状态是host内核层面的，
# 容器看不到独立的物理网口状态）：
#   ./scripts/monitor_lidar_link.sh [网口名，默认enP8p1s0] > /path/to/log &
set -eo pipefail

IFACE="${1:-enP8p1s0}"
echo "# monitor_lidar_link started iface=${IFACE} t0=$(date -Iseconds)"

last_state=""
while true; do
    line=$(ip -o link show "${IFACE}" 2>&1)
    # operstate在ip -o link输出的"state XXX"字段
    state=$(echo "$line" | grep -oP '(?<=state )\S+' || echo "UNKNOWN")
    if [ "$state" != "$last_state" ]; then
        echo "$(date -Iseconds) iface=${IFACE} state ${last_state:-<start>} -> ${state}"
        last_state="$state"
    fi
    sleep 1
done
