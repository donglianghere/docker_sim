#!/usr/bin/env bash
# 进地面站网页栈容器的命令行（gcs-gcs-1）。
#
#   ./enter_gcs.sh                   # 交互式 bash，进去就能跑 ros2 CLI / rviz2
#   ./enter_gcs.sh "ros2 topic list" # 跑一条就退出
#
# 2026-10-03 新建。这一层原来缺这一个——桌面 GCS/enter_gcs.sh 是直接软链到
# scripts/exec_gcs.sh 的，而那份在 scripts/ 下、**没有双击暂停 trap**，双击
# 运行时窗口会立刻关掉、看不见输出。另外三个 enter_*（sim/contestant/
# sound_light）都在本目录、都有 trap，只有 gcs 这一个是例外。现在补齐。
#
# 逻辑全部转发给 scripts/exec_gcs.sh——那份会按 PLANNER 重新推导
# RMW_IMPLEMENTATION（entrypoint.sh 运行时 export 的那个，docker exec 开的新
# shell 读不到），不补的话 PLANNER=ego_planner 时这个 shell 还是默认 FastDDS，
# 跟已切到 CycloneDDS 的真实 rosbridge/ros2 节点完全对不上话题。
set -eo pipefail
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
C=gcs-gcs-1
docker ps --format '{{.Names}}' | grep -qx "$C" || {
    echo "!! 容器 $C 没在跑" >&2
    echo "   先起它： $(dirname "${BASH_SOURCE[0]}")/start_gcs.sh" >&2
    exit 1
}
echo "== $C   网页 http://localhost:8080 =="
echo "   查话题加 --no-daemon（不加会读 ROS 2 daemon 的缓存，可能是别人起的）"
exec "$ROOT/scripts/exec_gcs.sh" "$@"
