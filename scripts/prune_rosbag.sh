#!/usr/bin/env bash
# 配合record_rosbag.sh：只保留最近KEEP_COUNT个bag块（默认12个×5分钟=
# 1小时滚动窗口），超过的从最老的开始整个目录删掉——"黑匣子"式，不是
# 无限增长。已经被save_incident.sh复制到别处（incidents/）的不受影响，
# 这里只清理/logs/rosbag/自己这一份滚动录制。
#
# 用法：跟record_rosbag.sh在同一个容器里常驻跑：
#   ./scripts/prune_rosbag.sh
set -eo pipefail

OUT_DIR="${OUT_DIR:-/logs/rosbag}"
KEEP_COUNT="${KEEP_COUNT:-12}"
CHECK_INTERVAL="${CHECK_INTERVAL:-60}"

echo "== prune_rosbag: 只保留 ${OUT_DIR} 下最近 ${KEEP_COUNT} 个bag块 =="
while true; do
  mapfile -t bags < <(ls -1dt "${OUT_DIR}"/bag_* 2>/dev/null)
  if [ "${#bags[@]}" -gt "$KEEP_COUNT" ]; then
    for ((i = KEEP_COUNT; i < ${#bags[@]}; i++)); do
      old="${bags[$i]}"
      rm -rf "$old"
      echo "[prune_rosbag] 删除过期录制: $old"
    done
  fi
  sleep "$CHECK_INTERVAL"
done
