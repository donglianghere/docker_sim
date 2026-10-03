#!/usr/bin/env bash
# 声光装置自检：让装置真的响一次、亮一次。
#
#   ./test_sound_light.sh                  # 默认序列：起飞 → 发现火情 → 任务完成
#   ./test_sound_light.sh --list           # 列出全部 20 个事件（名字 + RGB + 声音编号）
#   ./test_sound_light.sh --event 任务机投放灭火弹
#   ./test_sound_light.sh --sound 7        # 直接按声音编号 1~20
#   ./test_sound_light.sh --mute           # 熄灯静音
#   ./test_sound_light.sh --status         # 只看状态，不发声
#
# 原理：常驻程序订阅 /sound_light/request（std_msgs/String），**纯文本就行**
# ——事件名 / 声音编号 / mute。本脚本从已在跑的 contestant-sound-light 容器里
# 发，因为它身上已经有正确的 ROS_DOMAIN_ID=20 和 CYCLONEDDS_URI，不用另起容器
# 也不会跟仿真的 21 域搞混。
#
# 两个时间约束（常驻程序的参数，别和"没响"搞混）：
#   · 最小间隔 2.5s —— 连发比这更密，后面的会排队而不是立刻播，所以序列里 sleep 3
#   · 过期 15s      —— 排队超过 15 秒的请求会被丢掉
set -eo pipefail
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
C=contestant-sound-light
VOCAB="$ROOT/src/contest_sdk/contest_sdk/_sound_light_port.py"

die() { echo "!! $*" >&2; exit 1; }

# 容器里跑一条 ros2 命令。`source` 不能省：镜像里 ros2 不在默认 PATH 上。
in_c() { docker exec "$C" bash -lc "source /opt/ros/humble/setup.bash && $1"; }

# ---- 前置：容器在不在跑 ----
docker ps --format '{{.Names}}' | grep -qx "$C" || {
    echo "!! 声光常驻程序没在跑（容器 $C 不存在）" >&2
    echo "   先起它： $(dirname "${BASH_SOURCE[0]}")/start_sound_light_server.sh" >&2
    echo "   或双击桌面 SOUND/start_sound_light_server.sh" >&2
    die "已中止"
}

# ---- 状态：串口有没有打开 ----
show_status() {
    local raw
    # `--full-length`（-f）不能省：ros2 topic echo 默认把长字符串截到 128 字符后
    # 加省略号，状态 JSON 正好超过，截断后 json.loads 直接失败。
    raw="$(in_c 'timeout 10 ros2 topic echo --once --full-length /sound_light/status' 2>/dev/null \
           | sed -n "s/^data: '\(.*\)'$/\1/p" | head -1)"
    [ -n "$raw" ] || { echo "   （读不到 /sound_light/status，常驻程序可能刚起还没发状态）"; return 1; }
    python3 - "$raw" <<'PY'
import json, sys
try:
    d = json.loads(sys.argv[1])
except ValueError:
    print("   状态原文：" + sys.argv[1][:160]); sys.exit(0)
print(f"   串口 {d.get('port')}  已连接={d.get('connected')}  dry_run={d.get('dry_run')}")
print(f"   已播 {d.get('played')} 条  排队 {d.get('queued')}  丢弃 {d.get('dropped')}")
if d.get('error'):      print(f"   ⚠ 错误：{d['error']}")
if d.get('last_played'): print(f"   最近播放：{d['last_played']}")
sys.exit(0 if d.get('connected') else 1)
PY
}

list_events() {
    echo "可用事件（名字 -> R,G,B,声音编号,重复,间隔）："
    sed -n "/^SOUND_LIGHT_EVENTS *= *{/,/^}/s/^ *'\(.*\)': (\(.*\)),$/  \1 -> \2/p" "$VOCAB"
    echo
    echo "也可以直接 --sound 1..20 按编号放，或 --mute 熄灯静音。"
}

send() {   # send <文本> <说明>
    echo "-- 发送：$1   （$2）"
    in_c "timeout 15 ros2 topic pub --once /sound_light/request std_msgs/String 'data: $1'" \
        >/dev/null 2>&1 || die "发布失败"
}

# ---- 参数 ----
case "${1:-}" in
    --list)   list_events; exit 0 ;;
    --status) echo "声光常驻程序状态："; show_status || die "串口没打开——brltty 抢了 CH340？用 systemctl stop brltty"; exit 0 ;;
    --mute)   send "mute" "熄灯静音"; echo; echo "装置应已熄灯静音。"; exit 0 ;;
    --event)  [ -n "${2:-}" ] || die "--event 后面要跟事件名（--list 看可选）"
              grep -q "'$2':" "$VOCAB" || { echo "!! 没有这个事件：$2" >&2; echo >&2; list_events >&2; exit 2; }
              send "$2" "单个事件"; echo; echo "听到声音、看到灯色变化就算通过。"; exit 0 ;;
    --sound)  [ -n "${2:-}" ] || die "--sound 后面要跟 1~20"
              case "$2" in ''|*[!0-9]*) die "--sound 要是数字 1~20，收到 '$2'" ;; esac
              [ "$2" -ge 1 ] && [ "$2" -le 20 ] || die "--sound 范围是 1~20，收到 $2"
              send "$2" "声音编号 $2"; echo; echo "听到第 $2 号声音就算通过。"; exit 0 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    "")       ;;
    *)        die "未知参数：$1（用 --help 看用法）" ;;
esac

# ---- 默认：跑一小段序列 ----
echo "声光常驻程序状态："
show_status || die "串口没打开，先修这个——brltty 抢了 CH340？用 systemctl stop brltty"
echo
echo "开始自检序列，共 3 条。每条之间等 3 秒（常驻程序最小间隔是 2.5 秒）。"
echo "请看着装置：灯色应依次变成 蓝 → 红 → 绿，每次伴随一声提示音。"
echo
send "侦察机起飞"           "蓝 50,150,255 + 声音1"
sleep 3
send "侦察机发现地面火情"   "红 255,0,0 + 声音5"
sleep 3
send "侦察机任务完成"       "绿 0,255,0 + 声音12"
sleep 3
echo
echo "序列发完。再看一次状态（played 应该涨了 3）："
show_status || true
echo
echo "三次都听到声音、看到灯色变化 = 装置正常。"
echo "一次都没响 → 看 docker logs $C 里有没有\"发送：\"那一行："
echo "  有  = 常驻程序发出去了，问题在串口/装置那一端（接线、电源、波特率）"
echo "  没有 = 请求没到常驻程序，查域号（本脚本从容器里发，应该不会错）"
