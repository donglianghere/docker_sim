#!/usr/bin/env bash
# 地面检查 3：双机互见与链路质量。
#
#   ./c3_双机链路.sh            # 全查
#   ./c3_双机链路.sh --quick    # 跳过耗时的频率测量
#
# **不起飞。** 两机通电、飞行栈起着即可。
#
# 查四层，从下往上，坏在哪一层一眼看出来：
#   [1] 网络      —— ping 可达 + 丢包率
#   [2] 管控      —— 8890 /status（control_server 活着、容器状态、命名空间）
#   [3] DDS       —— 地面站能看到两机话题；**两机能不能看到对方**（编队的前提）
#   [4] 数据      —— uwb/pose_abs 的实际频率（僚机跟随就靠它）
#
# 为什么要专门测"两机互见"：编队靠的是僚机订阅长机的位姿。两机各自跟地面站
# 通不代表它们之间通——AP 下多播不可靠，靠的是 entrypoint 生成的静态 Peers
# （/tmp/docker_sim_cyclonedds.xml）。Peer 配反了的表现是"单飞都正常、一编队
# 就跟不上"，到现场才发现就晚了。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
QUICK=0
for a in "$@"; do
    case "$a" in
        --quick) QUICK=1 ;;
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "!! 未知参数：$a" >&2; exit 2 ;;
    esac
done
NX01_IP=192.168.2.101
NX02_IP=192.168.2.102
C=docker_sim-flight-stack-hw-1
IMAGE=contestant-sdk:latest

in_uav() {   # in_uav <ip> <命令>
    timeout 50 ssh -o ConnectTimeout=6 nvidia@"$1" \
        "docker exec $C bash -lc 'source /opt/ros/humble/setup.bash && $2'" 2>&1
}

echo "=============================================="
echo " [1/4] 网络：ping 20 包看丢包"
echo "=============================================="
for pair in "NX01:$NX01_IP" "NX02:$NX02_IP"; do
    ns="${pair%%:*}"; ip="${pair##*:}"
    r="$(timeout 40 ping -c 20 -i 0.2 -W 1 "$ip" 2>/dev/null | tail -2)"
    # 只取数字部分。原来用 ${loss%%.*} 截，拿到的是整串 "0% packet loss"，
    # 永远不等于 0，于是 0 丢包也被判成 △（2026-10-03 实测发现）。
    lossn="$(printf '%s' "$r" | grep -oE '[0-9]+(\.[0-9]+)?% packet loss' | grep -oE '^[0-9]+' | head -1)"
    avg="$(printf '%s' "$r" | sed -nE 's#.*= [0-9.]+/([0-9.]+)/.*#\1#p' | head -1)"
    rtt="$(printf '%s' "$r" | grep -oE 'min/avg/max[^=]*= [0-9./]+' | head -1)"
    if [ -z "$lossn" ]; then echo "  ✗ $ns ($ip) 不可达"; continue; fi
    mark=✓
    [ "$lossn" != 0 ] && mark=△
    # WiFi 局域网 avg RTT 正常几毫秒；几十毫秒以上说明链路在抢或信号弱
    [ -n "$avg" ] && [ "${avg%%.*}" -ge 30 ] 2>/dev/null && mark=△
    printf "  %s %-16s 丢包 %-4s  %s\n" "$mark" "$ns ($ip)" "${lossn}%" "$rtt"
done
echo "  丢包 >0、或 avg RTT ≥30ms，都要查 AP："
echo "  5G ch149/80MHz 是已验证的健康配置（txpower 的 3dBm 是 mt7925 假读数，别查它）"

echo
echo "=============================================="
echo " [2/4] 管控：8890 /status"
echo "=============================================="
FLIGHT_UP_COUNT=0
for pair in "NX01:$NX01_IP" "NX02:$NX02_IP"; do
    ns="${pair%%:*}"; ip="${pair##*:}"
    st="$(curl -s --noproxy '*' --max-time 8 "http://$ip:8890/status" 2>/dev/null || true)"
    printf '%s' "$st" | grep -q "'flight_up': True\|\"flight_up\": true" \
        && FLIGHT_UP_COUNT=$((FLIGHT_UP_COUNT+1))
    if [ -z "$st" ]; then
        echo "  ✗ $ns control_server(8890) 没响应——它是开机自启的，服务挂了"
        echo "      ssh nvidia@$ip 'sudo systemctl restart uav-control-server'"
        continue
    fi
    printf '%s' "$st" | python3 -c "
import sys,json
d=json.load(sys.stdin)
ns=d.get('namespace','?')
ok = '✓' if (d.get('flight_up') and d.get('vision_up')) else '△'
print(f\"  {ok} $ns  namespace={ns}  flight_up={d.get('flight_up')}  vision_up={d.get('vision_up')}\")
if ns != '$ns':
    print(f'      ✗ 命名空间不符！这个 IP 报的是 {ns}，期望 $ns —— .namespace 配反了')
" 2>/dev/null || echo "  △ $ns /status 解析失败：$(printf '%s' "$st" | head -c 100)"
done

echo
echo "=============================================="
echo " [3/4] DDS：地面站视角 + **两机互见**"
echo "=============================================="
# 飞行栈没起的话话题当然一条都没有，这时报"查 Peer/域号"是误导——先说清楚。
if [ "$FLIGHT_UP_COUNT" = 0 ]; then
    echo "  跳过：两机的 flight-stack 都没起（见 [2/4] 的 flight_up=False），"
    echo "  话题本来就不存在，这一层测不出链路问题。先起机载栈："
    echo "    curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \\"
    echo "         -d '{\"stack\":\"flight\",\"action\":\"up\",\"confirm\":true}' \\"
    echo "         http://192.168.2.101:8890/stack      # NX02 同理"
    echo "  或在地面站界面点按钮。起好之后重跑本脚本。"
    exit 0
fi
[ "$FLIGHT_UP_COUNT" = 1 ] && \
    echo "  ⚠ 只有一架的 flight-stack 起着——"两机互见"这一项测不了，下面会报 ✗"
echo "  地面站能看到的两机话题数（--no-daemon，不读缓存）："
source "$ROOT/scripts/contestant_network.sh"
contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" >/dev/null 2>&1 || true
gcs_view="$(timeout 70 docker run --rm --network host "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
    bash -lc 'source /opt/ros/humble/setup.bash
              timeout 25 ros2 topic list --no-daemon 2>/dev/null' 2>/dev/null)"
for ns in NX01 NX02; do
    n="$(printf '%s\n' "$gcs_view" | grep -c "^/$ns/" || true)"
    printf "    %s %s：%s 条\n" "$([ "${n:-0}" -gt 10 ] && echo ✓ || echo ✗)" "$ns" "${n:-0}"
done
[ "$(printf '%s\n' "$gcs_view" | grep -c '^/NX0' || true)" = 0 ] && \
    echo "    ✗ 一条都看不到——查地面站 ROS_DOMAIN_ID=20 和 gcs/cyclonedds_gcs.xml 的本机 IP"

echo
echo "  两机互见（编队的前提，单飞正常 ≠ 这一项正常）："
for pair in "NX01:$NX01_IP:NX02" "NX02:$NX02_IP:NX01"; do
    me="${pair%%:*}"; rest="${pair#*:}"; ip="${rest%%:*}"; peer="${rest##*:}"
    ping -c1 -W2 "$ip" >/dev/null 2>&1 || { echo "    ✗ $me 不可达"; continue; }
    n="$(in_uav "$ip" "timeout 25 ros2 topic list --no-daemon 2>/dev/null | grep -c '^/$peer/'" \
         | grep -E '^[0-9]+$' | tail -1)"
    printf "    %s %s 看到 %s 的话题：%s 条\n" \
        "$([ "${n:-0}" -gt 10 ] && echo ✓ || echo ✗)" "$me" "$peer" "${n:-0}"
    [ "${n:-0}" -le 10 ] && echo "        查 $me 上 /tmp/docker_sim_cyclonedds.xml 的 Peer 是不是 $peer 的 IP"
done

if [ "$QUICK" = 1 ]; then
    echo; echo "（--quick：跳过 [4/4] 频率测量）"; exit 0
fi
echo
echo "=============================================="
echo " [4/4] 数据：uwb/pose_abs 频率（僚机跟随就靠它）"
echo "=============================================="
for pair in "NX01:$NX01_IP" "NX02:$NX02_IP"; do
    ns="${pair%%:*}"; ip="${pair##*:}"
    ping -c1 -W2 "$ip" >/dev/null 2>&1 || { echo "  ✗ $ns 不可达"; continue; }
    hz="$(in_uav "$ip" "timeout 15 ros2 topic hz --no-daemon /$ns/uwb/pose_abs 2>/dev/null | head -2" \
          | grep -oE "average rate: [0-9.]+" | head -1)"
    printf "  %s %s  %s\n" "$([ -n "$hz" ] && echo ✓ || echo ✗)" "$ns" "${hz:-读不到（话题没在发？）}"
done
echo
echo "全部 ✓ 才算链路就绪。任一项 ✗ 都会在编队时表现成"僚机跟不上"。"
