#!/usr/bin/env bash
# 关掉**地面电脑上**一切真机相关的栈。
#
#   ./stop_all.sh              # 全关
#   ./stop_all.sh --check      # 只看会动什么，不执行
#   ./stop_all.sh --keep-gcs   # 保留网页栈
#   ./stop_all.sh --land       # 额外给两机发降落指令（转给 stop_real.sh）
#
# ⚠️ **只动地面电脑，一个机载容器都不碰。** 机载飞行栈/视觉栈归各机的
# control_server 管，生命周期比一次飞行长，重起还要重锁原点。要停机载的用
# 地面站界面或 POST /stack {"stack":"flight","action":"down","confirm":true}。
#
# ⚠️ **关掉这些不会让飞机降落。** 选手程序发的是任务级指令，掐掉之后机载
# ego_planner 会把手上最后一条轨迹执行完、然后悬停在那里。应急手段是
# **遥控器接管**。--land 会就地降落，落点不一定是起降垫，用前确认下方无人。
#
# 和 stop_real.sh 的分工：选手容器和监视交给它，本脚本补它不管的：
#   · run_test.sh 起的 real_test / real_test_monitor
#   · 声光栈   —— 共享资源，**只在它处于真机域 20 时才关**
#   · 网页栈   —— run_real.sh 必起它，所以这里也负责关（--keep-gcs 可留）
#   · 选手调试容器 contestant-sdk-shell —— 同样按域判断
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
SH="$ROOT/shell"
WANT_DOMAIN=20        # 真机域

CHECK=0; KEEP_GCS=0; PASS=()
for a in "$@"; do
    case "$a" in
        --check)    CHECK=1; PASS+=(--check) ;;
        --land)     PASS+=(--land) ;;
        --keep-gcs) KEEP_GCS=1 ;;
        -h|--help)  sed -n '2,28p' "$0"; exit 0 ;;
        *) echo "未知参数: $a" >&2; exit 2 ;;
    esac
done
say() { echo "  $*"; }
domain_of() {
    docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$1" 2>/dev/null \
      | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1
}
drop() {   # drop <容器名>
    docker ps -a --format '{{.Names}}' | grep -qx "$1" || { say "没有 $1"; return 0; }
    if [ "$CHECK" = 1 ]; then say "会删：$1"
    else docker rm -f "$1" >/dev/null 2>&1 && say "已删 $1" || say "⚠ 删 $1 失败"; fi
}

echo "=============================================="
echo " 关闭地面电脑上的真机相关栈$([ "$CHECK" = 1 ] && echo "（--check：只看不动）")"
echo " （机载容器一个都不碰）"
echo "=============================================="

echo "[A] 选手容器 / 监视 —— 交给 stop_real.sh"
"$HERE/stop_real.sh" "${PASS[@]}" 2>&1 | sed 's/^/  /'

echo "[B] run_test.sh 起的单机测试容器"
drop real_test
drop real_test_monitor

echo "[C] 声光栈（只在真机域 $WANT_DOMAIN 时关）"
c=contestant-sound-light
if docker ps --format '{{.Names}}' | grep -qx "$c"; then
    d="$(domain_of "$c")"
    if [ "$d" = "$WANT_DOMAIN" ]; then
        if [ "$CHECK" = 1 ]; then say "会关：$c（域 $d）"
        else
            # `printf '\n' |` 不能省：双击用的脚本末尾有 read 暂停，喂 /dev/null
            # 会让它返回假失败（2026-10-03 踩过）。
            printf '\n' | "$SH/stop_sound_light_server.sh" >/dev/null 2>&1 \
                && say "已关 $c（域 $d）" || say "⚠ 关 $c 失败，手动跑 shell/stop_sound_light_server.sh"
        fi
    else
        say "保留：$c 在域 $d，不是真机域 $WANT_DOMAIN —— 别人在用，不动"
    fi
else
    say "没在跑"
fi

echo "[D] 选手调试容器（只在真机域 $WANT_DOMAIN 时关）"
c=contestant-sdk-shell
if docker ps --format '{{.Names}}' | grep -qx "$c"; then
    d="$(domain_of "$c")"
    if [ "$d" = "$WANT_DOMAIN" ]; then
        if [ "$CHECK" = 1 ]; then say "会关：$c（域 $d）"
        else printf '\n' | "$SH/stop_contestant_shell.sh" >/dev/null 2>&1 \
                && say "已关 $c（域 $d）" || say "⚠ 关 $c 失败"; fi
    else
        say "保留：$c 在域 $d，不是真机域"
    fi
else
    say "没在跑"
fi

echo "[E] 网页栈"
if [ "$KEEP_GCS" = 1 ]; then
    say "按 --keep-gcs 保留"
elif docker ps --format '{{.Names}}' | grep -qE '^gcs-(gcs|backend)-1$'; then
    if [ "$CHECK" = 1 ]; then say "会关：gcs-gcs-1 + gcs-backend-1"
    else printf '\n' | "$SH/stop_gcs.sh" >/dev/null 2>&1 \
            && say "已关 网页栈" || say "⚠ 关网页栈失败，手动跑 shell/stop_gcs.sh"; fi
else
    say "没在跑"
fi

echo "[F] 剩下还在跑的容器"
left="$(docker ps --format '{{.Names}}' || true)"
if [ -z "$left" ]; then say "一个都没有"
else echo "$left" | sed 's/^/  /'; fi
echo
say "提醒：机载飞行栈/视觉栈没有被动过。飞机也不会因为这个脚本降落。"
echo "=============================================="
