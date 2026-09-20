#!/usr/bin/env bash
# tmux status-right专用——被tmux用`#(...)`周期性调用(mighty_sim.tmux.conf里
# `status-interval 1`，每秒重新执行一次)，必须快、必须不依赖docker exec(每次
# 起一个新容器内进程有几十~上百ms开销，虽然1秒一次问题不大，但没必要——
# scripts/status_monitor.py本来就已经在把结果写到宿主机侧可以直接读的
# runtime_logs/health_status.txt了，这里单纯cat它，一次stat+read，微秒级，
# 状态栏刷新不会有卡顿感)。
#
# 具体显示什么内容(2026-08-12起是每架飞机的实时飞行时长，之前是各子系统
# 健康图标)全部由status_monitor.py决定，这个脚本本身不做任何判定，只负责
# "读文件+过期检查+吐给tmux"，文件名/变量名仍叫HEALTH_FILE，只是历史沿用，
# 不代表内容还是健康状态。
set -eo pipefail
cd "$(dirname "$0")/.."
HEALTH_FILE="runtime_logs/health_status.txt"

if [ ! -f "$HEALTH_FILE" ]; then
    echo "#[fg=colour244]状态栏未启动(打开status窗口等它跑起来)#[fg=default]"
    exit 0
fi

# 写这个文件的进程是不是还活着——status_monitor.py每1秒重写一次这个文件，
# 如果文件mtime已经好几秒没更新，说明写它的那个进程(status窗口里那条
# docker exec链，或者status窗口本身被关掉了)挂了。这时候绝不能原样显示
# 文件里的旧内容，那样会让人误以为"一切正常"，其实只是监控自己不动了——
# 跟这次要解决的"NX02 SLAM停了没人发现"是同一类坑，只是换了一层，必须
# 单独防一下。
NOW=$(date +%s)
MTIME=$(stat -c %Y "$HEALTH_FILE" 2>/dev/null || echo 0)
AGE=$((NOW - MTIME))
if [ "$AGE" -gt 5 ]; then
    echo "#[fg=colour196,bold]⚠状态栏已断开(${AGE}s无更新，检查status窗口)#[fg=default,nobold]"
    exit 0
fi

cat "$HEALTH_FILE"
