#!/usr/bin/env bash
# 把某个服务的容器stdout持久化到宿主机文件——`docker compose logs`本身
# 读的是容器自己的日志存储，`docker compose down`把容器删掉之后这份
# 历史就跟着没了；这个脚本额外常驻tee一份到宿主机`runtime_logs/`目录，
# 容器怎么重建都不会丢，同时自己实现一个简化版`logrotate`（按文件大小
# 滚动、只留最近KEEP_FILES份），不引入新的系统依赖、也不会无限增长。
#
# 用`--tail=0`是因为不想每次这个脚本重启都把docker自己缓冲区里的历史
# 全量重新吐一遍进日志文件（造成重复）——只跟着接收"从现在起"的新增
# 输出，已经发生过的历史交给docker自己的json-file驱动（docker-compose.
# yml里配了max-size/max-file，这是第一层、生命周期更短但免维护的兜底）。
#
# 用法（宿主机侧跑，不是在容器里）：
#   ./scripts/tail_persist_logs.sh sim-world
#   ./scripts/tail_persist_logs.sh flight-stack-nx01
#   ./scripts/tail_persist_logs.sh flight-stack-nx02
set -eo pipefail
cd "$(dirname "$0")/.."

SERVICE="$1"
if [ -z "$SERVICE" ]; then
  echo "用法: $0 <compose服务名，如 sim-world / flight-stack-nx01 / flight-stack-nx02>" >&2
  exit 1
fi

OUT_DIR="runtime_logs/container_logs"
LOGFILE="${OUT_DIR}/${SERVICE}.log"
MAX_SIZE_BYTES="${MAX_SIZE_BYTES:-52428800}"   # 50MB
KEEP_FILES="${KEEP_FILES:-5}"                   # 50MB × 5 ≈ 250MB 单服务上限
mkdir -p "$OUT_DIR"

rotate_if_needed() {
  [ -f "$LOGFILE" ] || return 0
  local size
  size=$(stat -c%s "$LOGFILE" 2>/dev/null || echo 0)
  if [ "$size" -gt "$MAX_SIZE_BYTES" ]; then
    for ((i = KEEP_FILES - 1; i >= 1; i--)); do
      [ -f "${LOGFILE}.${i}" ] && mv "${LOGFILE}.${i}" "${LOGFILE}.$((i + 1))"
    done
    mv "$LOGFILE" "${LOGFILE}.1"
    # 超过KEEP_FILES份的最老一份直接丢弃
    [ -f "${LOGFILE}.$((KEEP_FILES + 1))" ] && rm -f "${LOGFILE}.$((KEEP_FILES + 1))"
    echo "[tail_persist_logs] ${SERVICE}: 日志超过$((MAX_SIZE_BYTES / 1048576))MB，已滚动"
  fi
}

# 后台每分钟检查一次是否要滚动，前台主进程负责持续追加写入
(
  while true; do
    sleep 60
    rotate_if_needed
  done
) &
rotator_pid=$!
trap 'kill "$rotator_pid" 2>/dev/null || true' EXIT

echo "== 持久化 ${SERVICE} 的stdout到 ${LOGFILE}（按${MAX_SIZE_BYTES}字节滚动，留${KEEP_FILES}份）=="
exec docker compose logs -f --no-color --tail=0 "$SERVICE" >> "$LOGFILE"
