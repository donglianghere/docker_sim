#!/usr/bin/env bash
# 一条命令：先docker compose up -d把sim-world/flight-stack-nx01/flight-stack-nx02
# 三个容器起在后台，再自动拉起watch_sim.sh（三个独立终端窗口，NX01/NX02两个
# 窗口内部各自再拆SLAM|规划|控制三个tmux窗格）。
# 用法: ./scripts/up_and_watch.sh
set -eo pipefail

cd "$(dirname "$0")/.."

echo "== docker compose up -d =="
docker compose up -d

exec ./scripts/watch_sim.sh
