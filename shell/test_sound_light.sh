#!/usr/bin/env bash
# 声光装置自检：让装置真的响一次、亮一次。
#
#   ./test_sound_light.sh                  # 默认序列：起飞 → 发现火情 → 任务完成
#   ./test_sound_light.sh --list           # 列出全部 20 个事件（名字 + RGB + 声音编号）
#   ./test_sound_light.sh --event 任务机投放灭火弹
#   ./test_sound_light.sh --sound 7        # 直接按声音编号 1~20
#   ./test_sound_light.sh --mute           # 熄灯静音
#   ./test_sound_light.sh --status         # 只看状态，不发声
#   ./test_sound_light.sh --sim            # 先断言容器在仿真域(21)，再跑序列
#   ./test_sound_light.sh --real           # 先断言容器在真机域(20)，再跑序列
#
# 关于仿真/真机：**本脚本自己不需要区分** ——它是 docker exec 进已在跑的容器
# 里发的，用的就是那个容器的域，不可能发到另一个域去。但声光常驻程序是
# **共享资源、同一时刻只能在一个域**（仿真21/真机20，见 start_sound_light_server.sh
# 的 --real），所以存在一个假就绪的坑：容器挂在 20 域时你测，装置照样响，可
# 仿真程序（21域）的事件根本到不了它。因此本脚本每次都把容器的域号打出来，
# 并提供 --sim/--real 做断言——要上哪个场景，就用对应的那个跑一遍。
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

# 跟 scripts/check_env.sh 的 domain_of 同一种取法，保持一致
domain_of() {
    docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$1" 2>/dev/null \
      | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1
}

# 容器在哪个域 → 这套声光此刻服务于哪个场景
show_domain() {
    local d; d="$(domain_of "$C")"
    case "$d" in
        20) echo "   域 20 → **真机**场景（仿真程序发的事件到不了）" ;;
        21) echo "   域 21 → **仿真**场景（真机程序发的事件到不了）" ;;
        "") echo "   ⚠ 读不到容器的 ROS_DOMAIN_ID" ;;
        *)  echo "   ⚠ 域 $d —— 既不是仿真(21)也不是真机(20)" ;;
    esac
}

# --sim / --real 的断言。不一致就给出和 check_env.sh 同样的修法
assert_mode() {   # assert_mode sim|real
    local want d
    [ "$1" = "real" ] && want=20 || want=21
    d="$(domain_of "$C")"
    [ "$d" = "$want" ] && { echo "   ✓ 域 $d 与 $1 模式一致"; return 0; }
    echo "!! 声光容器在域 ${d:-?}，但你要的是 $1 模式（应为 $want）" >&2
    echo "   声光常驻程序只能在一个域，换域要重起它：" >&2
    echo "     $(dirname "${BASH_SOURCE[0]}")/stop_sound_light_server.sh" >&2
    if [ "$1" = "real" ]; then
        echo "     $(dirname "${BASH_SOURCE[0]}")/start_sound_light_server.sh --real /dev/ttyUSB0" >&2
    else
        echo "     $(dirname "${BASH_SOURCE[0]}")/start_sound_light_server.sh /dev/ttyUSB0" >&2
    fi
    die "域不匹配，已中止"
}

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
# 用循环而不是 case "$1"：--sim/--real 是**断言**，要能和 --event/--sound/
# --mute/--status 组合（`--real --event X` = 先确认在真机域，再放那一个事件）。
# 第一版写成 case "$1" 的单分支，--real 命中后直接落到默认序列，后面的
# --event 被静默忽略——实测踩到。
WANT=""; ACTION=""; ARG=""
while [ $# -gt 0 ]; do
    case "$1" in
        --sim)     WANT=sim; shift ;;
        --real)    WANT=real; shift ;;
        --list)    ACTION=list; shift ;;
        --status)  ACTION=status; shift ;;
        --mute)    ACTION=mute; shift ;;
        --event)   ACTION=event; ARG="${2:-}"; [ -n "$ARG" ] || die "--event 后面要跟事件名（--list 看可选）"; shift 2 ;;
        --sound)   ACTION=sound; ARG="${2:-}"; [ -n "$ARG" ] || die "--sound 后面要跟 1~20"; shift 2 ;;
        -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
        *)         die "未知参数：$1（用 --help 看用法）" ;;
    esac
done

# --list 不碰容器，先处理掉
[ "$ACTION" = list ] && { list_events; exit 0; }

echo "声光常驻程序状态："
show_domain
[ -n "$WANT" ] && assert_mode "$WANT"

case "$ACTION" in
    status) show_status || die "串口没打开——brltty 抢了 CH340？用 systemctl stop brltty"; exit 0 ;;
    mute)   send "mute" "熄灯静音"; echo; echo "装置应已熄灯静音。"; exit 0 ;;
    event)  grep -q "'$ARG':" "$VOCAB" || { echo "!! 没有这个事件：$ARG" >&2; echo >&2; list_events >&2; exit 2; }
            show_status || die "串口没打开，先修这个"
            send "$ARG" "单个事件"; echo; echo "听到声音、看到灯色变化就算通过。"; exit 0 ;;
    sound)  case "$ARG" in ''|*[!0-9]*) die "--sound 要是数字 1~20，收到 '$ARG'" ;; esac
            [ "$ARG" -ge 1 ] && [ "$ARG" -le 20 ] || die "--sound 范围是 1~20，收到 $ARG"
            show_status || die "串口没打开，先修这个"
            send "$ARG" "声音编号 $ARG"; echo; echo "听到第 $ARG 号声音就算通过。"; exit 0 ;;
esac

# ---- 默认：跑一小段序列 ----
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
