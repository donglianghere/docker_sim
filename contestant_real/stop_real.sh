#!/usr/bin/env bash
# 真机急停 / 收尾：掐掉地面站这边的选手程序和监视，不动机载栈。
#
#   ./stop_real.sh            # 掐掉选手程序 + 监视（飞机保持当前状态）
#   ./stop_real.sh --land     # 额外给两机发降落指令（**要先确认下方无人**）
#   ./stop_real.sh --check    # 只看会动什么，不执行
#
# ⚠️ 读清楚这一段再用：
#   掐掉选手程序**不会让飞机降落**。选手程序发的是任务级指令，掐掉之后机载的
#   ego_planner 会把手上最后一条轨迹执行完、然后悬停在那里。真正的应急手段是
#   **遥控器接管**（PX4CTRL_NO_RC=false，飞行时遥控器必须在手）。
#   这个脚本的用处是"止住新指令"，不是"让飞机下来"。
#   --land 会发 TakeoffLand{LAND}，飞机就地降落——落点不一定是起降垫，
#   用之前先确认正下方没人没障碍。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
LOCK=/tmp/docker_sim_run_real.lock
C_LEADER=real_leader
C_FOLLOWER=real_follower
C_MONITOR=real_monitor
LEADER=NX01; FOLLOWER=NX02
IMAGE=contestant-sdk:latest

LAND=0; CHECK=0
for a in "$@"; do
    case "$a" in
        --land)  LAND=1 ;;
        --check) CHECK=1 ;;
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "未知参数: $a" >&2; exit 2 ;;
    esac
done
say() { echo "  $*"; }
run() { [ "$CHECK" = 1 ] && { say "[check] 会执行: $*"; return 0; }; "$@"; }

echo "=============================================="
echo " 真机收尾$([ "$CHECK" = 1 ] && echo "（--check：只看不动）")"
echo "=============================================="

# ---- 1. 还在跑的 run_real.sh ----
# 排除自己这条调用链上的所有祖先进程：pkill 按模式匹配会连自己的 bash -c
# 一起杀掉（本项目踩过三次），所以逐级往上收集 PID 再排除。
echo "[1/3] 还在跑的 run_real.sh"
mine=(); p=$$
while [ "$p" != "1" ] && [ -n "$p" ]; do mine+=("$p"); p="$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')"; done
is_mine() { local q; for q in "${mine[@]}"; do [ "$q" = "$1" ] && return 0; done; return 1; }
found=0
for pid in $(pgrep -f 'bash .*/run_real\.sh' 2>/dev/null); do
    kill -0 "$pid" 2>/dev/null || continue
    is_mine "$pid" && continue
    found=1; say "杀掉 run_real.sh (pid $pid)"; run kill -TERM "$pid" 2>/dev/null || true
done
[ "$found" = 0 ] && say "没有在跑的 run_real.sh"

# ---- 2. 选手容器与监视 ----
echo "[2/3] 选手容器与监视"
any=0
for c in "$C_LEADER" "$C_FOLLOWER" "$C_MONITOR"; do
    if docker ps -a --format '{{.Names}}' | grep -qx "$c"; then
        any=1; say "删除容器 $c"; run docker rm -f "$c" >/dev/null 2>&1 || true
    fi
done
[ "$any" = 0 ] && say "没有选手/监视容器"
[ -f "$LOCK" ] && { say "释放锁 $LOCK"; run rm -f "$LOCK"; }

# ---- 3. 可选：发降落 ----
echo "[3/3] 降落指令"
if [ "$LAND" = 0 ]; then
    say "未加 --land，不发降落指令"
    say "飞机会停在最后一条轨迹的终点悬停——要下来请用**遥控器接管**"
else
    say "发 TakeoffLand{LAND} 给 $LEADER 和 $FOLLOWER"
    # shellcheck source=/dev/null
    source "$ROOT/scripts/contestant_network.sh"
    if contestant_net_args real "$ROOT" "$HOME/.cache/contest_sdk"; then
        for ns in "$LEADER" "$FOLLOWER"; do
            run docker run --rm --network host "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
                bash -lc "source /opt/ros/humble/setup.bash && \
                    ros2 topic pub --once /$ns/takeoff_land quadrotor_msgs/msg/TakeoffLand \
                    '{takeoff_land_cmd: 2}' >/dev/null 2>&1" || say "  !! $ns 降落指令发送失败"
            say "  已发给 $ns"
        done
        say "⚠️ 降落指令已发出，盯住飞机；遥控器保持在手"
    else
        say "!! 真机网络参数生成失败，降落指令没发出去——改用遥控器"
    fi
fi

echo
echo "机载飞行栈/视觉栈**未动**（它们跑在飞机上，由各机 control_server 管）。"
echo "要停机载栈：走地面站网页，或 curl 各机的 8890 端点。"
