#!/usr/bin/env bash
# 地面检查 2：舵机接线/参数核对 + 两机行程一致性（迁移清单 G1 / G3）。
#
#   ./c2_舵机.sh                 # 两机：先核参数，再走行程
#   ./c2_舵机.sh NX02            # 只测 NX02
#   ./c2_舵机.sh both --params   # 只核飞控参数，一下都不动舵机
#
# **不起飞、不解锁。** 飞控 COM_PREARM_MODE=2 之后舵机不解锁也能动（电机不转）。
#
# ⚠️ 为什么先核参数再动舵机：SDK 的 SERVO_CONFIG 里 **NX01 那一路从来没在真机
# 上核对过**（源码注释原话："接线和飞控参数还没在NX01真机上核对过，暂按NX02
# 的第一路照搬"），而且写着"接错口会驱动别的输出"。所以在给 MAIN7 发 PWM 之前
# 必须先确认它真的被配成舵机输出，否则可能驱动到别的东西上去。
#
# 核的三项（源码注释点名要确认的）：
#   PWM_MAIN_FUNC7 = 301     MAIN7 = Actuator Set1（SDK 舵机1）
#   PWM_MAIN_FUNC9 = 302     MAIN9 = Actuator Set2（SDK 舵机2，只有 NX02 有）
#   PWM_MAIN_TIM2  = 50      该组频率 50 Hz（舵机要 50Hz，不是电机那种高频）
#   COM_PREARM_MODE = 2      不解锁也允许动执行器
#
# 行程：800 -> 1400 -> 2000 -> 1400 -> 800，每档停 1.5 秒，**人眼看**。
#   800 = 抓紧 / 2000 = 松开（2026-09-21 真机已验，NX02）
# 两机一致性靠**同一组 PWM 下看两机动作幅度是否相同**——舵机没有位置反馈，
# 软件读不到角度，这一项只能人判，脚本负责把两机按同样的节奏驱动。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

TARGET=both; PARAMS_ONLY=0
for a in "$@"; do
    case "$a" in
        NX01|NX02|both) TARGET="$a" ;;
        --params) PARAMS_ONLY=1 ;;
        -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "!! 未知参数：$a" >&2; exit 2 ;;
    esac
done
case "$TARGET" in
    NX01) LIST=("NX01:192.168.2.101:1") ;;
    NX02) LIST=("NX02:192.168.2.102:1,2") ;;
    both) LIST=("NX01:192.168.2.101:1" "NX02:192.168.2.102:1,2") ;;
esac

IMAGE=contestant-sdk:latest
source "$ROOT/scripts/contestant_network.sh"
contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" \
    || { echo "!! 真机网络参数生成失败" >&2; exit 1; }

# 在地面站的选手容器里跑——选手程序本来就是在地面站驱动机上舵机的，
# 走的是 /{ns}/mavros/cmd/command 服务，不需要 ssh 上飞机。
sdk_run() {   # sdk_run <python代码>
    timeout 90 docker run --rm --network host "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
        bash -lc "source /opt/ros/humble/setup.bash && python3 -c \"$1\"" 2>&1
}

for item in "${LIST[@]}"; do
    ns="${item%%:*}"; rest="${item#*:}"; ip="${rest%%:*}"; servos="${rest##*:}"
    echo "=============================================="
    echo " $ns ($ip)  舵机 $servos"
    echo "=============================================="
    ping -c1 -W2 "$ip" >/dev/null 2>&1 || { echo "  !! 不可达，跳过"; continue; }

    echo "[1] 飞控参数核对（接错口会驱动别的输出，所以先核这个）"
    want="PWM_MAIN_FUNC7=301 PWM_MAIN_TIM2=50 COM_PREARM_MODE=2"
    [ "$servos" = "1,2" ] && want="$want PWM_MAIN_FUNC9=302"
    bad=0
    for kv in $want; do
        k="${kv%%=*}"; v="${kv##*=}"
        got="$(sdk_run "
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import ParamGet
rclpy.init(); n=Node('pg')
c=n.create_client(ParamGet, '/$ns/mavros/param/get')
print('NOSVC') if not c.wait_for_service(timeout_sec=8.0) else None
if c.service_is_ready():
    r=ParamGet.Request(); r.param_id='$k'
    f=c.call_async(r); rclpy.spin_until_future_complete(n,f,timeout_sec=8.0)
    res=f.result()
    if res is None or not res.success: print('FAIL')
    else: print(int(res.value.integer) if res.value.integer else float(res.value.real))
n.destroy_node(); rclpy.shutdown()
" | grep -vE "^\[|WARN|INFO" | tail -1)"
        case "$got" in
            "$v"|"$v.0") printf "    ✓ %-16s = %s\n" "$k" "$got" ;;
            NOSVC)       printf "    ⚠ %-16s 读不到（mavros param 服务没响应）\n" "$k"; bad=1 ;;
            FAIL|"")     printf "    ⚠ %-16s 读失败\n" "$k"; bad=1 ;;
            *)           printf "    ✗ %-16s = %s，期望 %s\n" "$k" "$got" "$v"; bad=1 ;;
        esac
    done
    if [ "$bad" != 0 ]; then
        echo "    ✗ 参数没全对——**先不要动舵机**。用 QGC 或 mavros param set 改好再测。"
        echo "      PWM_MAIN_FUNCn=300+Set号；舵机那一组的 PWM_MAIN_TIMn 必须是 50。"
        continue
    fi
    [ "$PARAMS_ONLY" = 1 ] && { echo "    （--params：只核参数，不动舵机）"; continue; }

    echo "[2] 走行程 800 -> 1400 -> 2000 -> 1400 -> 800，每档停 1.5 秒"
    echo "    **盯着机械机构看**：到位是否干脆、有无卡滞、两机幅度是否一致"
    sdk_run "
import time
from contest_sdk import DroneSDK
sdk = DroneSDK(namespace='$ns', role='recon', teammate_namespace='$ns')
print('可用舵机:', dict(sdk.servos))
for s in [int(x) for x in '$servos'.split(',')]:
    for pwm in (800, 1400, 2000, 1400, 800):
        sdk.set_servo(s, pwm); print(f'  舵机{s} -> {pwm}'); time.sleep(1.5)
sdk.shutdown()
" | grep -vE "^\[|WARN|INFO|^$" | sed 's/^/    /'
    echo
done
echo "两机一致性：同一组 PWM 下两机动作幅度应当相同。舵机没有位置反馈，"
echo "软件读不到角度，这一项只能人判——幅度差得明显就去核对机械行程和限位。"
