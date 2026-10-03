#!/usr/bin/env bash
# 关掉**一切仿真相关**的栈。
#
#   ./stop_all.sh            # 全关
#   ./stop_all.sh --check    # 只看会动什么，不执行
#   ./stop_all.sh --purge    # 额外删掉 Exited 的历史容器（转给 stop.sh）
#
# 和 stop.sh 的分工：重活全交给它（run.sh 进程、容器内 monitor/referee、
# sim_leader/sim_follower、compose down 三个仿真容器），本脚本只补它不管的：
#   · 声光栈   —— 共享资源，**只在它处于仿真域 21 时才关**。挂在真机域 20 的
#                 不是"仿真相关"，动它会打断别人正在用的那一套。
#   · 选手调试容器 contestant-sdk-shell —— 同样按域判断。
#
# 网页栈（gcs-gcs-1 / gcs-backend-1）**不关**：run.sh 本来就不起它，它是跨
# 仿真/真机共用的观察面，关掉只会碍事。要关用 shell/stop_gcs.sh。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
SH="$ROOT/shell"
WANT_DOMAIN=21        # 仿真域

CHECK=0; PASS=()
for a in "$@"; do
    case "$a" in
        --check) CHECK=1; PASS+=(--check) ;;
        --purge) PASS+=(--purge) ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "未知参数: $a" >&2; exit 2 ;;
    esac
done
say() { echo "  $*"; }
domain_of() {
    docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$1" 2>/dev/null \
      | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1
}

echo "=============================================="
echo " 关闭一切仿真相关栈$([ "$CHECK" = 1 ] && echo "（--check：只看不动）")"
echo "=============================================="

echo "[A] 仿真栈 / 选手栈 / 监视 / 裁判 —— 交给 stop.sh"
"$HERE/stop.sh" "${PASS[@]}" 2>&1 | sed 's/^/  /'

echo "[B] 声光栈（只在仿真域 $WANT_DOMAIN 时关）"
c=contestant-sound-light
if docker ps --format '{{.Names}}' | grep -qx "$c"; then
    d="$(domain_of "$c")"
    if [ "$d" = "$WANT_DOMAIN" ]; then
        if [ "$CHECK" = 1 ]; then say "会关：$c（域 $d）"
        else
            # `printf '\n' |` 不能省：那个脚本是给双击用的，末尾有
            # `read -n 1 ... 按任意键关闭窗口` 的 EXIT trap。喂 /dev/null 会让
            # read 拿到 EOF 返回 1、在 set -e 下报假失败（2026-10-03 踩过）。
            printf '\n' | "$SH/stop_sound_light_server.sh" >/dev/null 2>&1 \
                && say "已关 $c（域 $d）" || say "⚠ 关 $c 失败，手动跑 shell/stop_sound_light_server.sh"
        fi
    else
        say "保留：$c 在域 $d，不是仿真域 $WANT_DOMAIN —— 别人在用，不动"
    fi
else
    say "没在跑"
fi

echo "[C] 选手调试容器（只在仿真域 $WANT_DOMAIN 时关）"
c=contestant-sdk-shell
if docker ps --format '{{.Names}}' | grep -qx "$c"; then
    d="$(domain_of "$c")"
    if [ "$d" = "$WANT_DOMAIN" ]; then
        if [ "$CHECK" = 1 ]; then say "会关：$c（域 $d）"
        else printf '\n' | "$SH/stop_contestant_shell.sh" >/dev/null 2>&1 \
                && say "已关 $c（域 $d）" || say "⚠ 关 $c 失败"; fi
    else
        say "保留：$c 在域 $d，不是仿真域"
    fi
else
    say "没在跑"
fi

echo "[D] 剩下还在跑的容器"
left="$(docker ps --format '{{.Names}}' || true)"
if [ -z "$left" ]; then say "一个都没有"
else echo "$left" | sed 's/^/  /'; say "（网页栈按设计保留；要关用 shell/stop_gcs.sh）"; fi
echo "=============================================="
