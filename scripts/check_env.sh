#!/usr/bin/env bash
# 仿真/真机互不残留检查。两边共用，run_real.sh 会自动调它。
#
#   ./scripts/check_env.sh real    # 准备跑真机：确认没有仿真残余
#   ./scripts/check_env.sh sim     # 准备跑仿真：确认没有真机残余
#   ./scripts/check_env.sh         # 只报告当前状态，不判对错
#
# 为什么需要：仿真和真机**主要靠 ROS_DOMAIN_ID 隔离**（仿真 21 / 真机 20），
# 话题层面不会串。但有三类东西是共享的、隔离不了：
#   ① 声光常驻程序——只有一个容器、只能在一个域。它在哪个域，另一个模式的
#      播报就全部静默失效（不报错）。
#   ② CPU/GPU/X11——Gazebo 很重，仿真在跑的时候真机那边的选手程序和监视
#      会被挤，表现为指令延迟、监视掉帧。
#   ③ 地面站网页栈 gcs-gcs-1——它自己也有域，网页上的"仿真/真机模式"开关会
#      改写 gcs/cyclonedds_gcs.xml 并带着它一起切。
# 容器名和锁是分开的（sim_leader/real_leader、docker_sim_run.lock /
# docker_sim_run_real.lock），不会互相拆，这部分不用查。
set -uo pipefail
MODE="${1:-}"
SIM_CONTAINERS="docker_sim-sim-world-1 docker_sim-flight-stack-nx01-1 docker_sim-flight-stack-nx02-1 sim_leader sim_follower"
REAL_CONTAINERS="real_leader real_follower real_monitor"
bad=0
say() { echo "  $*"; }

running() { docker ps --format '{{.Names}}' | grep -qx "$1"; }
domain_of() {
    docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$1" 2>/dev/null \
      | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1
}

echo "=============================================="
echo " 环境互不残留检查${MODE:+（目标模式：$MODE）}"
echo "=============================================="

echo "[1/3] 另一侧的容器"
if [ "$MODE" = "real" ]; then
    hit=0
    for c in $SIM_CONTAINERS; do
        running "$c" && { say "✗ 仿真容器还在跑：$c"; hit=1; bad=1; }
    done
    [ "$hit" = 0 ] && say "✓ 没有在跑的仿真容器"
    say "  （Exited 状态的仿真容器不占 CPU，不用管）"
elif [ "$MODE" = "sim" ]; then
    hit=0
    for c in $REAL_CONTAINERS; do
        running "$c" && { say "✗ 真机容器还在跑：$c"; hit=1; bad=1; }
    done
    [ "$hit" = 0 ] && say "✓ 没有在跑的真机选手容器"
    say "  （两架飞机可以保持开机——它们在域 20，跟仿真的域 21 天然隔离）"
else
    say "仿真容器在跑：$(for c in $SIM_CONTAINERS; do running "$c" && echo -n "$c "; done || true)"
    say "真机容器在跑：$(for c in $REAL_CONTAINERS; do running "$c" && echo -n "$c "; done || true)"
fi

echo "[2/3] 声光常驻程序（共享资源，只能在一个域）"
if running contestant-sound-light; then
    d="$(domain_of contestant-sound-light)"
    want=""; [ "$MODE" = "real" ] && want=20; [ "$MODE" = "sim" ] && want=21
    if [ -z "$want" ]; then
        say "当前域=$d"
    elif [ "$d" = "$want" ]; then
        say "✓ 域=$d，与 $MODE 模式一致"
    else
        say "✗ 域=$d，但 $MODE 模式要 $want——播报会全部静默失效"
        say "  修：./stop_sound_light_server.sh 然后"
        [ "$MODE" = "real" ] && say "      ./start_sound_light_server.sh --real /dev/ttyUSB0" \
                             || say "      ./start_sound_light_server.sh /dev/ttyUSB0"
        bad=1
    fi
else
    say "△ 没在跑——播报不会出声（不拦住飞行，但任务通报会缺）"
fi

echo "[3/3] 地面站网页栈"
if running gcs-gcs-1; then
    d="$(domain_of gcs-gcs-1)"
    say "gcs-gcs-1 域=$d$([ -n "$MODE" ] && { [ "$MODE" = real ] && [ "$d" = 20 ] && echo "（与 real 一致）"; [ "$MODE" = sim ] && [ "$d" = 21 ] && echo "（与 sim 一致）"; })"
    say "  它只用于观察，域不对不影响飞行，但网页上看到的数据会是另一个模式的"
else
    say "没在跑（纯观察用，不影响飞行）"
fi

echo
if [ -n "$MODE" ]; then
    [ "$bad" = 0 ] && echo "结论：环境干净，可以跑 $MODE" || echo "结论：有残留，按上面的提示处理后再跑"
fi
exit $bad
