#!/usr/bin/env bash
# 一键清理：把这套仿真相关的东西全部停掉，回到干净状态。
#
#   ./stop.sh            # 停仿真：选手容器 + 监视 + 裁判 + 三个仿真容器
#   ./stop.sh --purge    # 另外删掉本套仿真**已退出**的历史容器（不可逆）
#   ./stop.sh --check    # 只检查不动手，看看现在还剩什么
#
# **不碰** contestant-sound-light：那是声光常驻程序，本来就该一直开着。
# **不碰**其他项目的容器（gcs-*、contest_task_* 这些），只认本套仿真的名字。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # 选手目录
ROOT="$(cd "$HERE/.." && pwd)"
LOCK=/tmp/docker_sim_run.lock
SIMWORLD=docker_sim-sim-world-1
FSNX01=docker_sim-flight-stack-nx01-1
PURGE=0; CHECK=0
for a in "$@"; do
    case "$a" in
        --purge) PURGE=1 ;;
        --check) CHECK=1 ;;
        -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
        *) echo "未知参数: $a" >&2; exit 2 ;;
    esac
done
say() { echo "  $*"; }

echo "=============================================="
echo " 清理仿真$([ "$CHECK" = 1 ] && echo "（--check：只看不动）")"
echo "=============================================="

# ---- 1. 还在跑的 run.sh ----
# 不能用 pkill -f 'run.sh'：那个模式会匹配到**本脚本自己的命令行**和调用它的
# 父 shell，2026-10-01 就这么把自己的 shell 打死过（退出码 144）。
# 只认两个来源：锁文件里记的 PID，和 pgrep 到的 run.sh，且都要排掉自己和祖先。
echo "[1/4] 还在跑的 run.sh"
mine=(); p=$$
while [ "$p" != "1" ] && [ -n "$p" ]; do mine+=("$p"); p="$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')"; done
is_mine() { local q; for q in "${mine[@]}"; do [ "$q" = "$1" ] && return 0; done; return 1; }
cands="$( { [ -f "$LOCK" ] && cat "$LOCK"; pgrep -f 'bash .*/run\.sh' 2>/dev/null; } | sort -u)"
found=0
for pid in $cands; do
    [ -z "$pid" ] && continue
    kill -0 "$pid" 2>/dev/null || continue
    is_mine "$pid" && continue
    found=1
    cmd="$(ps -o args= -p "$pid" 2>/dev/null | cut -c1-70)"
    if [ "$CHECK" = 1 ]; then say "还在跑：PID $pid  $cmd"
    else kill -9 "$pid" 2>/dev/null && say "已杀 PID $pid  $cmd"; fi
done
[ "$found" = 0 ] && say "没有"

# ---- 2. 容器里的监视和裁判 ----
# 它们是 docker exec -d 起的，compose down 会连带干掉，但先停一下更干净，
# 也便于 --check 看清现状。
echo "[2/4] 容器内的监视 / 裁判"
for pair in "$FSNX01:monitor.py" "$SIMWORLD:referee.py"; do
    c="${pair%%:*}"; proc="${pair##*:}"
    docker ps --format '{{.Names}}' | grep -qx "$c" || continue
    if docker exec "$c" pgrep -f "$proc" >/dev/null 2>&1; then
        if [ "$CHECK" = 1 ]; then say "$c 里还在跑 $proc"
        else docker exec "$c" pkill -f "$proc" >/dev/null 2>&1; say "已停 $c 的 $proc"; fi
    fi
done

# ---- 3. 选手容器 + 整套仿真 ----
echo "[3/4] 容器"
live="$(docker ps --format '{{.Names}}' | grep -E '^sim_(leader|follower)$|^docker_sim-' || true)"
if [ -z "$live" ]; then say "没有在跑的仿真容器"
elif [ "$CHECK" = 1 ]; then echo "$live" | sed 's/^/  在跑：/'
else
    docker rm -f sim_leader sim_follower >/dev/null 2>&1 && say "已删 选手容器"
    ( cd "$ROOT" && WORLD_ENV=sample_room docker compose down --timeout 20 2>&1 \
        | grep -E 'Removed' | sed 's/^/  /' )
fi
if [ "$PURGE" = 1 ] && [ "$CHECK" = 0 ]; then
    gone="$(docker ps -a --filter status=exited --format '{{.Names}}' \
            | grep -E '^sim_(leader|follower)$|^docker_sim-' || true)"
    [ -n "$gone" ] && { echo "$gone" | xargs -r docker rm >/dev/null 2>&1; \
        echo "$gone" | sed 's/^/  已删历史容器：/'; }
fi

# ---- 4. 核对 ----
echo "[4/4] 核对残留"
# gzserver/px4/mavros/pt4ctrl 都跑在仿真容器里。容器还在时它们本来就该在，
# 只有容器都没了还剩着才算真残留（那种情况是容器被强杀、进程没跟着走）。
still="$(docker ps --format '{{.Names}}' | grep -cE '^docker_sim-' || true)"
left="$(pgrep -af 'gzserver|gzclient|rviz2|px4_sitl|/bin/px4|mavros_node|pt4ctrl' 2>/dev/null \
        | grep -v "stop\.sh" || true)"
if [ "${still:-0}" != "0" ]; then
    say "仿真容器还在跑（$still 个），容器内进程不算残留"
    [ "$CHECK" = 0 ] && say "⚠ 没清干净，再跑一次 ./stop.sh"
elif [ -n "$left" ]; then
    echo "$left" | sed 's/^/  ⚠ 真残留（容器已没、进程还在）：/'
    say "这种要手动 kill，正常 compose down 不该留下它们"
else
    say "无残留进程"
fi
keep="$(docker ps --format '{{.Names}}' | grep -v -E '^sim_|^docker_sim-' || true)"
[ -n "$keep" ] && echo "$keep" | sed 's/^/  保留（不属于本套仿真）：/'
echo "=============================================="
