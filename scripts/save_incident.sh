#!/usr/bin/env bash
# 手动触发的"留证"：事情刚发生（炸机、规划失败、异常行为）的时候立刻
# 跑一下，把最近几个滚动bag块（覆盖事发前后这段时间）+ 两机的姿态/推力
# 文本日志，复制到 /logs/incidents/<时间戳>_<描述>/ 长期保留——不会被
# record_rosbag.sh配合prune_rosbag.sh的滚动窗口清理掉（那边只清理
# /logs/rosbag/自己，这里复制出去的是独立的另一份）。
#
# 用法（在挂载了/logs的容器里跑，比如flight-stack-nx01）：
#   ./scripts/save_incident.sh "NX02悬停不动报fopt17647"
set -eo pipefail

DESC="${1:-incident}"
# 描述里可能有空格/中文，替换成下划线，避免目录名出问题
SAFE_DESC=$(echo "$DESC" | tr ' /' '__')
TS=$(date +%Y%m%d_%H%M%S)
DEST="/logs/incidents/${TS}_${SAFE_DESC}"
mkdir -p "$DEST"

# 最近3个bag块（按record_rosbag.sh默认5分钟/块，覆盖最近15分钟），
# 足够包含"事情发生前后"，不需要整个滚动窗口都复制一遍。
RECENT_N="${RECENT_N:-3}"
mapfile -t recent_bags < <(ls -1dt /logs/rosbag/bag_* 2>/dev/null | head -n "$RECENT_N")
if [ "${#recent_bags[@]}" -eq 0 ]; then
  echo "!! /logs/rosbag/ 下没有找到任何bag块，record_rosbag.sh是不是没在跑？"
else
  for b in "${recent_bags[@]}"; do
    cp -r "$b" "$DEST/"
  done
fi

# 两机的姿态/推力文本日志各留一份完整拷贝（这个不滚动删除，直接整份copy）
for ns in NX01 NX02; do
  if [ -f "/logs/${ns}/attitude_thrust_debug.log" ]; then
    cp "/logs/${ns}/attitude_thrust_debug.log" "$DEST/${ns}_attitude_thrust_debug.log"
  fi
done

echo "== 已保存到: $DEST =="
ls -la "$DEST"
