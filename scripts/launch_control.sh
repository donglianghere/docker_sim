#!/usr/bin/env bash
# tmux "launch"窗口用的起飞控制脚本——docker compose up之后飞机不再自动
# 起飞爬升，会先原地悬停在起飞位置（已经解锁、切进OFFBOARD，只是setpoint
# 钉在当前位置不动），等这个脚本里按一下回车，才会给两个flight-stack
# 容器都touch /tmp/takeoff_go，飞机检测到这个文件才真正开始往上爬
# （见patches/ros2_px4_stack_takeoff_gate.patch）。
set -eo pipefail

CONTAINERS=(docker_sim-flight-stack-nx01-1 docker_sim-flight-stack-nx02-1)

while true; do
    echo "=================================================="
    echo "  起飞口令控制台"
    echo "  飞机已经解锁/进入OFFBOARD，正悬停在起飞位置等待起飞口令"
    echo "  按回车键，让 NX01/NX02 同时开始起飞爬升"
    echo "=================================================="
    read -r -p "按回车起飞（或 Ctrl-C 取消）... "

    any_ok=0
    for c in "${CONTAINERS[@]}"; do
        if docker exec "$c" touch /tmp/takeoff_go 2>/dev/null; then
            echo "== $c: 起飞口令已下达 =="
            any_ok=1
        else
            echo "!! $c: 容器不存在或没起来，跳过 !!"
        fi
    done

    if [[ "$any_ok" -eq 1 ]]; then
        echo ""
        echo "起飞口令已下达，飞机应该开始爬升了。"
        break
    else
        echo ""
        echo "一个容器都没找到，容器是不是还没起来？稍后重试或 Ctrl-C 退出。"
        echo ""
    fi
done

echo ""
echo "这个窗口可以留着，也可以切到别的窗口看飞行状态"
echo "（Shift+方向键切窗口，或点状态栏窗口名，或 Ctrl-a + 数字键）。"
# 保持前台不退出，避免remain-on-exit那句"process已退出"提示掩盖掉上面
# 的确认信息
tail -f /dev/null
