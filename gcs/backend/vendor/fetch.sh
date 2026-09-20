#!/usr/bin/env bash
# ----------------------------------------------------------------------------
# 手动预取脚本——docker CLI静态二进制+docker-compose插件，在宿主机上跑，
# 不在docker build里跑，跟根目录fetch_sources.sh同样的思路："build阶段
# 只做COPY，不现场下载"，这样即使Dockerfile其它层的缓存失效（比如改了
# apt包列表、改了requirements.txt），这两个大文件（合计约130MB）也不会
# 跟着被迫重新下载——2026-08-25新增，起因是"来回build都要重新下载这两个
# 东西，网络慢、要等好几分钟"这个真实反馈，见DEBUG_JOURNAL.md同日期记录。
#
# 用法：
#   export http_proxy=http://127.0.0.1:7897/
#   export https_proxy=$http_proxy
#   ./fetch.sh
#
# 幂等：文件已存在就跳过，不重新下载；要强制刷新先手动删掉对应文件。
# 版本号（docker-26.1.4 / compose v2.29.7）要跟gcs/backend/Dockerfile里
# COPY的文件名保持一致，两边都要改；这两个文件本身不进git（见
# ../../.gitignore），每台开发机第一次用之前要跑一次这个脚本。
# ----------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"

DOCKER_TGZ="docker-26.1.4.tgz"
COMPOSE_BIN="docker-compose-linux-x86_64"

if [[ -f "$DOCKER_TGZ" ]]; then
  echo "== $DOCKER_TGZ 已存在，跳过 =="
else
  echo "== 下载 $DOCKER_TGZ =="
  curl -fsSL -o "$DOCKER_TGZ" \
    "https://download.docker.com/linux/static/stable/x86_64/docker-26.1.4.tgz"
fi

if [[ -f "$COMPOSE_BIN" ]]; then
  echo "== $COMPOSE_BIN 已存在，跳过 =="
else
  echo "== 下载 $COMPOSE_BIN =="
  curl -fsSL -o "$COMPOSE_BIN" \
    "https://github.com/docker/compose/releases/download/v2.29.7/docker-compose-linux-x86_64"
fi

echo "== 完成，文件大小 =="
ls -la "$DOCKER_TGZ" "$COMPOSE_BIN"
