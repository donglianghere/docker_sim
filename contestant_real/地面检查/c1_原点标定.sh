#!/usr/bin/env bash
# 地面检查 1：原点标定重复性（迁移清单 A4）。
#
#   ./c1_原点标定.sh                 # 两机各锁 5 次
#   ./c1_原点标定.sh NX01            # 只测 NX01
#   ./c1_原点标定.sh both 8          # 两机各锁 8 次
#
# **不起飞。** 飞机放在起降垫上静止、通电、飞行栈起着就行。
#
# 为什么必须先过这一项：world->odom 的平移是靠 set_origin_from_uwb 一次性锁死
# 的。2026-09-24 那次里程计没收敛就锁，偏差 0.47/0.67 米**整场飞行都带着**，
# 而且表现成"航点都差一点"而不是"标定错了"，很难反查。后面所有跟坐标有关的
# 测试（t1~t5、编队、落点核对）都建立在这一项之上。
#
# 判据：连续多次锁定，锁出来的平移量应该**基本重合**。
#   · 标准差 < 0.05 m        = 好
#   · 0.05 ~ 0.15 m          = 勉强，再等里程计稳一会儿重测
#   · > 0.15 m 或有次数失败  = 不要飞，先查 UWB/里程计
# 节点自己的门限是 max_std_dev_m=0.15（单次采样窗口内），这里测的是**多次之间**
# 的一致性，是更严的判据。
set -uo pipefail
TARGET="${1:-both}"
N="${2:-5}"
case "$TARGET" in
    NX01) HOSTS=("NX01:192.168.2.101") ;;
    NX02) HOSTS=("NX02:192.168.2.102") ;;
    both) HOSTS=("NX01:192.168.2.101" "NX02:192.168.2.102") ;;
    -h|--help) sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "!! 不认识的目标：$TARGET（可选 NX01 / NX02 / both）" >&2; exit 2 ;;
esac
case "$N" in ''|*[!0-9]*) echo "!! 次数要是数字，收到 '$N'" >&2; exit 2 ;; esac
[ "$N" -ge 2 ] || { echo "!! 至少锁 2 次才能看一致性" >&2; exit 2; }

C=docker_sim-flight-stack-hw-1
# 容器里跑一条 ros2 命令。域号/RMW 都是容器级 env，只需 source。
in_uav() {   # in_uav <ip> <命令>
    timeout 40 ssh -o ConnectTimeout=6 nvidia@"$1" \
        "docker exec $C bash -lc 'source /opt/ros/humble/setup.bash && $2'" 2>&1
}

for pair in "${HOSTS[@]}"; do
    ns="${pair%%:*}"; ip="${pair##*:}"
    echo "=============================================="
    echo " $ns ($ip)  连锁 $N 次"
    echo "=============================================="

    ping -c1 -W2 "$ip" >/dev/null 2>&1 || { echo "  !! 不可达，跳过"; continue; }
    if ! in_uav "$ip" 'true' >/dev/null 2>&1; then
        echo "  !! 容器 $C 里跑不了命令——飞行栈起了吗？"; continue; fi

    # 锁之前先看里程计稳不稳。没收敛就锁是这一项要防的头号问题。
    echo "[0] 锁之前：里程计在不在发、飞机是不是静止"
    in_uav "$ip" "timeout 12 ros2 topic hz --no-daemon /$ns/dlio/odom_node/odom 2>/dev/null | head -2" \
        | grep -iE "average|rate" | sed 's/^/    里程计 /' || echo "    ⚠ 读不到里程计频率"

    ok=0; fail=0; rm -f /tmp/_c1_$ns.txt
    for i in $(seq 1 "$N"); do
        out="$(in_uav "$ip" "timeout 25 ros2 service call /$ns/set_origin_from_uwb std_srvs/srv/Trigger")"
        if echo "$out" | grep -q "success=True"; then
            # 锁定结果落在飞机的 /tmp/uwb_origin_{ns}.json
            j="$(timeout 20 ssh -o ConnectTimeout=6 nvidia@"$ip" \
                 "docker exec $C cat /tmp/uwb_origin_$ns.json" 2>/dev/null)"
            xy="$(printf '%s' "$j" | python3 -c '
import sys,json
try:
    d=json.load(sys.stdin)
except Exception:
    print(""); raise SystemExit
for kx,ky in (("x","y"),("origin_x","origin_y"),("tx","ty")):
    if kx in d and ky in d:
        print(f"{float(d[kx]):.4f} {float(d[ky]):.4f}"); break
else:
    print("")
' 2>/dev/null)"
            if [ -n "$xy" ]; then
                echo "$xy" >> /tmp/_c1_$ns.txt
                printf "  第 %d/%d 次：success  平移 (%s)\n" "$i" "$N" "$(echo $xy | tr ' ' ',')"
                ok=$((ok+1))
            else
                printf "  第 %d/%d 次：success，但读不出平移量（json 字段名变了？）\n" "$i" "$N"
                echo "    原文：$(printf '%s' "$j" | head -c 150)"
                ok=$((ok+1))
            fi
        else
            printf "  第 %d/%d 次：**失败**\n" "$i" "$N"
            echo "$out" | grep -iE "message|error" | head -2 | sed 's/^/    /'
            fail=$((fail+1))
        fi
        sleep 3   # 节点采样窗口 sample_window_sec=2.0，留点余量
    done

    echo
    echo "[结果] $ns：成功 $ok 次，失败 $fail 次"
    [ "$fail" -gt 0 ] && echo "  ✗ 有失败——不要飞，先查 UWB 是否在发、飞机是否真静止"
    if [ -s /tmp/_c1_$ns.txt ]; then
        python3 - /tmp/_c1_$ns.txt <<'PY'
import sys, statistics as st
xs, ys = [], []
for line in open(sys.argv[1]):
    a = line.split()
    if len(a) == 2:
        xs.append(float(a[0])); ys.append(float(a[1]))
if len(xs) < 2:
    print("  样本不足 2 个，无法算一致性"); raise SystemExit
sx, sy = st.stdev(xs), st.stdev(ys)
rx, ry = max(xs)-min(xs), max(ys)-min(ys)
print(f"  平移均值  x={st.mean(xs):+.4f}  y={st.mean(ys):+.4f}")
print(f"  标准差    x={sx:.4f}  y={sy:.4f}   （m）")
print(f"  极差      x={rx:.4f}  y={ry:.4f}   （m）")
worst = max(sx, sy)
if worst < 0.05:
    print(f"  ✓ 一致性好（最大标准差 {worst:.4f} < 0.05 m）")
elif worst < 0.15:
    print(f"  △ 勉强（最大标准差 {worst:.4f}，在 0.05~0.15 m）——再等里程计稳一会儿重测")
else:
    print(f"  ✗ 不一致（最大标准差 {worst:.4f} > 0.15 m）——不要飞")
    print("    查：UWB 两个串口都在发吗；雷达出点云吗；飞机是不是真的没动")
PY
    fi
    rm -f /tmp/_c1_$ns.txt
    echo
done
echo "最后一次锁定的值就是飞行时生效的值。确认 origin_locked：" 
echo "  ros2 topic echo /<NS>/origin_locked --once --no-daemon"
