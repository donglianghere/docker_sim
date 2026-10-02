#!/usr/bin/env bash
# 机载视觉的开关与体检。
#
#   ./vision_real.sh status   # 两机四路的状态
#   ./vision_real.sh up       # 两机两路都起 YOLO（含 AprilTag 并行）
#   ./vision_real.sh down     # 两机都停
#
# 为什么需要这个：机载检测节点**不随飞行栈自启，也不持久化**——容器重启后
# 不会恢复上次的模式。手工敲的话是两机×两路共四条 curl，很容易漏一路，
# 而漏掉的后果是静默的：话题根本不存在（不是发空数组），wait_for_detection()
# 会一直等到超时。run_real.sh 只做**检查**不替你起，因为"自动起视觉"跟
# "起飞前人工确认"是矛盾的。
set -uo pipefail
LEADER_IP=192.168.2.101
FOLLOWER_IP=192.168.2.102
CMD="${1:-status}"

post() {  # post <ip> <json>
    curl -s --noproxy '*' --max-time 20 -X POST -H 'Content-Type: application/json' \
         -d "$2" "http://$1:8890/vision/mode" 2>/dev/null
}

case "$CMD" in
  up)
    for ip in "$LEADER_IP" "$FOLLOWER_IP"; do
        echo "--- $ip ---"
        for cam in cam0 cam1; do
            r=$(post "$ip" "{\"cam\":\"$cam\",\"mode\":\"yolo\"}")
            ok=$(echo "$r" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("ok"))' 2>/dev/null)
            msg=$(echo "$r" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("message",""))' 2>/dev/null)
            printf '  %-5s %s  %s\n' "$cam" "$([ "$ok" = True ] && echo ✓ || echo ✗)" "$msg"
            sleep 2
        done
    done
    echo
    echo "起完等约 10 秒再跑 ./vision_real.sh status 确认（节点要加载 TensorRT 引擎）"
    ;;
  down)
    for ip in "$LEADER_IP" "$FOLLOWER_IP"; do
        r=$(post "$ip" '{"cam":"all","mode":"stop"}')
        echo "--- $ip: $(echo "$r" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("message",""))' 2>/dev/null)"
    done
    ;;
  status)
    for ip in "$LEADER_IP" "$FOLLOWER_IP"; do
        st=$(curl -s --noproxy '*' --max-time 8 "http://$ip:8890/status" 2>/dev/null)
        if [ -z "$st" ]; then echo "--- $ip: control_server 无响应"; continue; fi
        ns=$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("namespace",""))' 2>/dev/null)
        vu=$(echo "$st" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("vision_up"))' 2>/dev/null)
        echo "--- $ip ($ns)  vision容器=$vu"
        # 容器在不等于节点在：容器跑的是 sleep infinity，节点要外部拉起
        n=$(ssh -o ConnectTimeout=5 -o BatchMode=yes "nvidia@$ip" \
            'docker exec docker_sim-vision-stack-1 ps -eo stat,comm --no-headers 2>/dev/null | grep yolo | grep -vc Z' 2>/dev/null || echo '?')
        echo "    存活检测节点: $n 个（应为 2：前视 + 下视）"
    done
    echo
    echo "--- 话题侧（最终判据，从选手容器看）---"
    ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    # shellcheck source=/dev/null
    source "$ROOT/scripts/contestant_network.sh"
    if contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk" >/dev/null 2>&1; then
        timeout 90 docker run --rm --network host "${CONTESTANT_NET_ARGS[@]}" contestant-sdk:latest \
          bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null; sleep 25
                    timeout 20 ros2 topic list --no-daemon 2>/dev/null \
                    | grep -E "vision/detections|image_raw|camera_info" | sort' 2>/dev/null \
          | grep -v '^== ' | sed 's/^/    /'
    else
        echo "    （真机网络参数生成失败，跳过话题检查）"
    fi
    ;;
  *) sed -n '2,14p' "$0"; exit 2 ;;
esac
